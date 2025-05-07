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
package com.irurueta.navigation.inertial.calibration.bias;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
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
import java.util.GregorianCalendar;

import static org.junit.jupiter.api.Assertions.*;

class BodyMagneticFluxDensityBiasEstimatorTest implements BodyMagneticFluxDensityBiasEstimatorListener {

    private static final int N_SAMPLES = 100000;

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

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-9;

    private int start;

    private int bodyMagneticFluxDensityAdded;

    private int reset;

    @Test
    void testConstructor1() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor2() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor3() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor4() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor5() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(),
                nedPosition.getLongitudeAngle(),
                nedPosition.getHeightDistance());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor6() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor7() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor8() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor9() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor10() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor11() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor12() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(),
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor13() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor14() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor15() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var randomizer = new UniformRandomizer();
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor16() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefPosition, ecefFrame.getECEFPosition());
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor17() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor18() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor19() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor20() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor21() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor22() throws IOException {
        final var randomizer = new UniformRandomizer();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC2, nedFrame.getCoordinateTransformation());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor23() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor24() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor25() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), year,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor26() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), year,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor27() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor28() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, year, 
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor29() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var randomizer = new UniformRandomizer();
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor30() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor31() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor32() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor33() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor34() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor35() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor36() throws IOException {
        final var randomizer = new UniformRandomizer();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor37() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor38() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor39() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), date,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor40() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), date,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor41() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor42() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, date, 
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor43() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var randomizer = new UniformRandomizer();
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor44() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor45() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor46() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), year,
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor47() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), year,
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor48() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor49() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, year, 
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor50() throws IOException {
        final var randomizer = new UniformRandomizer();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(year, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor51() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, year, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor52() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), year, magneticModel,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor53() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), year,
                magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor54() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), year,
                magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor55() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year, magneticModel, 
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor56() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date(createTimestamp(randomizer)));
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, year, 
                magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor57() throws IOException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var randomizer = new UniformRandomizer();
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor58() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor59() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor60() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), date,
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor61() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), date,
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor62() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor63() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefFrame.getECEFPosition(), cbn, date, 
                magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor64() throws IOException {
        final var randomizer = new UniformRandomizer();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(date, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor65() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(cbn, date, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final var nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor66() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitude(), nedPosition.getLongitude(), nedPosition.getHeight(), date, magneticModel,
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor67() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeight(), date,
                magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor68() throws IOException {
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                nedPosition.getLatitudeAngle(), nedPosition.getLongitudeAngle(), nedPosition.getHeightDistance(), date,
                magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor69() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date, magneticModel, 
                this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testConstructor70() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var date = new Date(createTimestamp(randomizer));
        final var year = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var magneticModel = new WorldMagneticModel();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(
                ecefFrame.getECEFPosition(), cbn, date, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final var ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC1, LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(ecefC2, LARGE_ABSOLUTE_ERROR));
        final var nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1, LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final var biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final var biasX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final var biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final var biasY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final var biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final var biasZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    void testGetSetTimeInterval() throws LockedException, IOException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetTimeIntervalAsTime() throws LockedException, IOException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();

        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set new value
        final var time2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setTimeInterval(time2);

        // check
        final var time3 = estimator.getTimeIntervalAsTime();
        final var time4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertEquals(time3, time4);
    }

    @Test
    void testGetSetEcefPosition1() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final var ecefPosition3 = ecefFrame.getECEFPosition();
        estimator.setEcefPosition(ecefPosition3);

        // check
        final var ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final var ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    void testGetSetEcefPosition2() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final var ecefPosition3 = ecefFrame.getECEFPosition();
        final var x = ecefPosition3.getX();
        final var y = ecefPosition3.getY();
        final var z = ecefPosition3.getZ();
        estimator.setEcefPosition(x, y, z);

        // check
        final var ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final var ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    void testGetSetEcefPosition3() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final var ecefPosition3 = ecefFrame.getECEFPosition();
        final var x = ecefPosition3.getDistanceX();
        final var y = ecefPosition3.getDistanceY();
        final var z = ecefPosition3.getDistanceZ();
        estimator.setEcefPosition(x, y, z);

        // check
        final var ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final var ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    void testGetSetEcefPosition4() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final var ecefPosition3 = ecefFrame.getECEFPosition();
        final var point = ecefPosition3.getPosition();
        estimator.setEcefPosition(point);

        // check
        final var ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final var ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    void testGetEcefFrame() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(ecefPosition, cbn);

        // check
        final var ecefFrame1 = estimator.getEcefFrame();
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetNedFrame() throws IOException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);

        final var nedFrame = new NEDFrame(nedPosition, cbn);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn);

        // check
        final var nedFrame1 = estimator.getNedFrame();
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);

        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(nedFrame2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition1() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var nedPosition = nedFrame.getPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final var nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition3 = createPosition(randomizer);

        estimator.setNedPosition(nedPosition3);

        // check
        final var nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final var nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition2() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var nedPosition = nedFrame.getPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final var nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition3 = createPosition(randomizer);
        final var latitude = nedPosition3.getLatitude();
        final var longitude = nedPosition3.getLongitude();
        final var height = nedPosition3.getHeight();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final var nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition3() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var nedPosition = nedFrame.getPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final var nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition3 = createPosition(randomizer);
        final var latitude = nedPosition3.getLatitudeAngle();
        final var longitude = nedPosition3.getLongitudeAngle();
        final var height = nedPosition3.getHeight();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final var nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition4() throws IOException, LockedException {
        final var nedFrame = new NEDFrame();
        final var nedPosition = nedFrame.getPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final var nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var nedPosition3 = createPosition(randomizer);
        final var latitude = nedPosition3.getLatitudeAngle();
        final var longitude = nedPosition3.getLongitudeAngle();
        final var height = nedPosition3.getHeightDistance();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final var nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetEcefC() throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var ecefC2 = estimator.getEcefC();
        assertEquals(ecefC1, ecefC2);
        final var ecefC3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC3);
        assertEquals(ecefC1, ecefC3);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame2 = new NEDFrame(cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefC4 = ecefFrame2.getCoordinateTransformation();

        estimator.setEcefC(ecefC4);

        // check
        final var ecefC5 = estimator.getEcefC();
        assertEquals(ecefC4, ecefC5);
        final var ecefC6 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC6);
        assertEquals(ecefC4, ecefC5);
    }

    @Test
    void testGetSetNedC() throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedC1 = nedFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var nedC2 = estimator.getNedC();
        assertEquals(nedC1, nedC2);
        final var nedC3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC3);
        assertEquals(nedC1, nedC3);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();

        estimator.setNedC(cbn);

        // check
        final var nedC4 = estimator.getNedC();
        assertEquals(cbn, nedC4);
        final var nedC5 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC5);
        assertEquals(cbn, nedC5);
    }

    @Test
    void testSetNedPositionAndNedOrientation1() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);

        estimator.setNedPositionAndNedOrientation(nedPosition2, cbn);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetNedPositionAndNedOrientation2() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var latitude = nedPosition2.getLatitude();
        final var longitude = nedPosition2.getLongitude();
        final var height = nedPosition2.getHeight();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, cbn);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetNedPositionAndNedOrientation3() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var latitude = nedPosition2.getLatitudeAngle();
        final var longitude = nedPosition2.getLongitudeAngle();
        final var height = nedPosition2.getHeight();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, cbn);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetNedPositionAndNedOrientation4() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var latitude = nedPosition2.getLatitudeAngle();
        final var longitude = nedPosition2.getLongitudeAngle();
        final var height = nedPosition2.getHeightDistance();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, cbn);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation1() throws IOException, InvalidSourceAndDestinationFrameTypeException, 
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setEcefPositionAndEcefOrientation(ecefPosition2, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetEcefPositionAndEcefOrientation2() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();

        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetEcefPositionAndEcefOrientation3() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var x = ecefPosition2.getDistanceX();
        final var y = ecefPosition2.getDistanceY();
        final var z = ecefPosition2.getDistanceZ();

        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetEcefPositionAndEcefOrientation4() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var point = ecefPosition2.getPosition();

        estimator.setEcefPositionAndEcefOrientation(point, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetNedPositionAndEcefOrientation1() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(nedPosition2, ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetNedPositionAndEcefOrientation2() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var latitude = nedPosition2.getLatitude();
        final var longitude = nedPosition2.getLongitude();
        final var height = nedPosition2.getHeight();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetNedPositionAndEcefOrientation3() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var latitude = nedPosition2.getLatitudeAngle();
        final var longitude = nedPosition2.getLongitudeAngle();
        final var height = nedPosition2.getHeight();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetNedPositionAndEcefOrientation4() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();
        final var latitude = nedPosition2.getLatitudeAngle();
        final var longitude = nedPosition2.getLongitudeAngle();
        final var height = nedPosition2.getHeightDistance();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    void testSetEcefPositionAndNedOrientation1() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();

        estimator.setEcefPositionAndNedOrientation(ecefPosition2, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetEcefPositionAndNedOrientation2() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();

        estimator.setEcefPositionAndNedOrientation(x, y, z, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetEcefPositionAndNedOrientation3() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var x = ecefPosition2.getDistanceX();
        final var y = ecefPosition2.getDistanceY();
        final var z = ecefPosition2.getDistanceZ();

        estimator.setEcefPositionAndNedOrientation(x, y, z, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testSetEcefPositionAndNedOrientation4() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var nedFrame1 = new NEDFrame();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();

        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition2 = createPosition(randomizer);
        final var nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var point = ecefPosition2.getPosition();

        estimator.setEcefPositionAndNedOrientation(point, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetYear() throws IOException, LockedException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var year1 = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var date = new Date(createTimestamp(randomizer));
        final var year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        estimator.setYear(year2);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    void testSetTime1() throws IOException, LockedException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var year1 = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var date = new Date(createTimestamp(randomizer));
        final var year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(date);

        estimator.setTime(date);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    void testSetTime2() throws IOException, LockedException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final var year1 = BodyMagneticFluxDensityBiasEstimator.convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var date = new Date(createTimestamp(randomizer));
        final var year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(date);
        final var calendar = new GregorianCalendar();
        calendar.setTime(date);

        estimator.setTime(calendar);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    void testGetSetMagneticModel() throws IOException, LockedException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertNull(estimator.getMagneticModel());

        // set new value
        final var magneticModel = new WorldMagneticModel();
        estimator.setMagneticModel(magneticModel);

        // check
        assertSame(magneticModel, estimator.getMagneticModel());
    }

    @Test
    void testGetSetListener() throws IOException, LockedException {
        final var estimator = new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddBodyMagneticFluxDensityAndReset() throws InvalidSourceAndDestinationFrameTypeException, IOException,
            LockedException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var cnb = generateBodyC(randomizer);
        final var cbn = cnb.inverseAndReturnNew();
        final var nedPosition = createPosition(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        // Expected body magnetic flux density for a static body at provided
        // location, orientation and timestamp
        final var earthB = wmmEstimator.estimate(nedPosition, timestamp);
        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var estimator = new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, timestamp, this);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyMagneticFluxDensityAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.isRunning());

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        BodyMagneticFluxDensity bodyMagneticFluxDensity = null;
        final var lastBodyMagneticFluxDensity = new BodyMagneticFluxDensity();
        for (var i = 0; i < N_SAMPLES; i++) {
            if (estimator.getLastBodyMagneticFluxDensity(lastBodyMagneticFluxDensity)) {
                assertEquals(estimator.getLastBodyMagneticFluxDensity(), lastBodyMagneticFluxDensity);
                assertEquals(bodyMagneticFluxDensity, lastBodyMagneticFluxDensity);
            }

            bodyMagneticFluxDensity = generateMeasure(truthMagnetic, hardIron, mm, noiseRandomizer);

            estimator.addBodyMagneticFluxDensity(bodyMagneticFluxDensity);

            assertTrue(estimator.getLastBodyMagneticFluxDensity(lastBodyMagneticFluxDensity));
            assertEquals(lastBodyMagneticFluxDensity, bodyMagneticFluxDensity);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyMagneticFluxDensityAdded);
        assertEquals(0, reset);

        final var biasBx = estimator.getBiasX();
        final var biasBy = estimator.getBiasY();
        final var biasBz = estimator.getBiasZ();

        assertEquals(hardIron[0], biasBx, ABSOLUTE_ERROR);
        assertEquals(hardIron[1], biasBy, ABSOLUTE_ERROR);
        assertEquals(hardIron[2], biasBz, ABSOLUTE_ERROR);

        final var biasBx1 = new MagneticFluxDensity(biasBx, MagneticFluxDensityUnit.TESLA);
        final var biasBx2 = estimator.getBiasXAsMagneticFluxDensity();
        final var biasBx3 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasBx3);

        assertEquals(biasBx1, biasBx2);
        assertEquals(biasBx1, biasBx3);

        final var biasBy1 = new MagneticFluxDensity(biasBy, MagneticFluxDensityUnit.TESLA);
        final var biasBy2 = estimator.getBiasYAsMagneticFluxDensity();
        final var biasBy3 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasBy3);

        assertEquals(biasBy1, biasBy2);
        assertEquals(biasBy1, biasBy3);

        final var biasBz1 = new MagneticFluxDensity(biasBz, MagneticFluxDensityUnit.TESLA);
        final var biasBz2 = estimator.getBiasZAsMagneticFluxDensity();
        final var biasBz3 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasBz3);

        assertEquals(biasBz1, biasBz2);
        assertEquals(biasBz1, biasBz3);

        final var biasTriad1 = estimator.getBiasTriad();
        assertEquals(biasBx, biasTriad1.getValueX(), 0.0);
        assertEquals(biasBy, biasTriad1.getValueY(), 0.0);
        assertEquals(biasBz, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final var biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);

        final var varianceBx = estimator.getVarianceX();
        final var varianceBy = estimator.getVarianceY();
        final var varianceBz = estimator.getVarianceZ();

        final var standardDeviationBx = estimator.getStandardDeviationX();
        final var standardDeviationBy = estimator.getStandardDeviationY();
        final var standardDeviationBz = estimator.getStandardDeviationZ();

        final var avgStdB = (standardDeviationBx + standardDeviationBy + standardDeviationBz) / 3.0;

        assertEquals(avgStdB, estimator.getAverageStandardDeviation(), 0.0);
        assertEquals(Math.sqrt(varianceBx), standardDeviationBx, 0.0);
        assertEquals(Math.sqrt(varianceBy), standardDeviationBy, 0.0);
        assertEquals(Math.sqrt(varianceBz), standardDeviationBz, 0.0);

        final var standardDeviationBx1 = new MagneticFluxDensity(standardDeviationBx, MagneticFluxDensityUnit.TESLA);
        final var standardDeviationBx2 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        final var standardDeviationBx3 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(standardDeviationBx3);

        assertEquals(standardDeviationBx1, standardDeviationBx2);
        assertEquals(standardDeviationBx1, standardDeviationBx3);

        final var standardDeviationBy1 = new MagneticFluxDensity(standardDeviationBy, MagneticFluxDensityUnit.TESLA);
        final var standardDeviationBy2 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        final var standardDeviationBy3 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(standardDeviationBy3);

        assertEquals(standardDeviationBy1, standardDeviationBy2);
        assertEquals(standardDeviationBy1, standardDeviationBy3);

        final var standardDeviationBz1 = new MagneticFluxDensity(standardDeviationBz, MagneticFluxDensityUnit.TESLA);
        final var standardDeviationBz2 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        final var standardDeviationBz3 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(standardDeviationBz3);

        assertEquals(standardDeviationBz1, standardDeviationBz2);
        assertEquals(standardDeviationBz1, standardDeviationBz3);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(standardDeviationBx, stdTriad1.getValueX(), 0.0);
        assertEquals(standardDeviationBy, stdTriad1.getValueY(), 0.0);
        assertEquals(standardDeviationBz, stdTriad1.getValueZ(), 0.0);
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);

        assertEquals(avgStdB, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStdB1 = estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(avgStdB1.getValue().doubleValue(), avgStdB, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStdB1.getUnit());
        final var avgStdB2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStdB2);
        assertEquals(avgStdB1, avgStdB2);

        final var psdBx = estimator.getPsdX();
        final var psdBy = estimator.getPsdY();
        final var psdBz = estimator.getPsdZ();

        final var timeInterval = estimator.getTimeInterval();
        assertEquals(varianceBx * timeInterval, psdBx, 0.0);
        assertEquals(varianceBy * timeInterval, psdBy, 0.0);
        assertEquals(varianceBz * timeInterval, psdBz, 0.0);

        final var rootPsdBx = estimator.getRootPsdX();
        final var rootPsdBy = estimator.getRootPsdY();
        final var rootPsdBz = estimator.getRootPsdZ();

        assertEquals(Math.sqrt(psdBx), rootPsdBx, 0.0);
        assertEquals(Math.sqrt(psdBy), rootPsdBy, 0.0);
        assertEquals(Math.sqrt(psdBz), rootPsdBz, 0.0);

        final var expectedRootPsd = Math.sqrt(MAGNETOMETER_NOISE_STD * MAGNETOMETER_NOISE_STD * timeInterval);
        assertEquals(expectedRootPsd, rootPsdBx, SMALL_ABSOLUTE_ERROR);
        assertEquals(expectedRootPsd, rootPsdBy, SMALL_ABSOLUTE_ERROR);
        assertEquals(expectedRootPsd, rootPsdBz, SMALL_ABSOLUTE_ERROR);

        final var avgPsdB = estimator.getAvgPsd();
        final var expectedPsdB = (psdBx + psdBy + psdBz) / 3.0;
        assertEquals(expectedPsdB, avgPsdB, 0.0);

        final var rootPsdB = estimator.getRootPsd();
        assertEquals(Math.sqrt(psdBx + psdBy + psdBz), rootPsdB, 0.0);
        assertEquals(Math.sqrt(rootPsdBx * rootPsdBx + rootPsdBy * rootPsdBy + rootPsdBz * rootPsdBz), rootPsdB,
                SMALL_ABSOLUTE_ERROR);

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());

        final var expectedB1 = estimator.getExpectedBodyMagneticFluxDensity();
        final var expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertTrue(truthMagnetic.equals(expectedB1, SMALL_ABSOLUTE_ERROR));
        assertTrue(truthMagnetic.equals(expectedB2, SMALL_ABSOLUTE_ERROR));

        // reset
        assertTrue(estimator.reset());

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertFalse(estimator.isRunning());
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        assertEquals(0.0, estimator.getBiasY(), 0.0);
        assertEquals(0.0, estimator.getBiasZ(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(1, reset);

        assertFalse(estimator.reset());
        assertEquals(1, reset);
    }

    @Override
    public void onStart(final BodyMagneticFluxDensityBiasEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onBodyMagneticFluxDensityAdded(final BodyMagneticFluxDensityBiasEstimator estimator) {
        if (bodyMagneticFluxDensityAdded == 0) {
            checkLocked(estimator);
        }
        bodyMagneticFluxDensityAdded++;
    }

    @Override
    public void onReset(final BodyMagneticFluxDensityBiasEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        start = 0;
        bodyMagneticFluxDensityAdded = 0;
        reset = 0;
    }

    private static void checkLocked(final BodyMagneticFluxDensityBiasEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        final var time = new Time(0.0, TimeUnit.SECOND);
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(time));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition((ECEFPosition) null));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition((Point3D) null));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null, null, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefC(null));
        assertThrows(LockedException.class, () -> estimator.setNedC(null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                null, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                null, null, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                (ECEFPosition) null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                (Point3D) null, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(null, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                null, null, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                (ECEFPosition) null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                (Point3D) null, null));
        assertThrows(LockedException.class, () -> estimator.setYear(0.0));
        assertThrows(LockedException.class, () -> estimator.setTime((Date) null));
        assertThrows(LockedException.class, () -> estimator.setTime((GregorianCalendar) null));
        assertThrows(LockedException.class, () -> estimator.setMagneticModel(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
    }

    private static BodyMagneticFluxDensity generateMeasure(
            final BodyMagneticFluxDensity truthMagnetic, final double[] hardIron, final Matrix softIron,
            final GaussianRandomizer noiseRandomizer) {

        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic;
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
