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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.gnss.GNSSBiasesGenerator;
import com.irurueta.navigation.gnss.GNSSConfig;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.gnss.GNSSMeasurementsGenerator;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class INSGNSSLooselyCoupledKalmanFilteredEstimatorTest implements
        INSGNSSLooselyCoupledKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final int MIN_NUM_SAT = 4;
    private static final int MAX_NUM_SAT = 10;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_SAT_HEIGHT = 150000.0;
    private static final double MAX_SAT_HEIGHT = 800000.0;

    private static final double MIN_SAT_VELOCITY_VALUE = -3500.0;
    private static final double MAX_SAT_VELOCITY_VALUE = 3500.0;

    private static final double MIN_MASK_ANGLE_DEGREES = 15.0;
    private static final double MAX_MASK_ANGLE_DEGREES = 20.0;

    private static final double POSITION_ERROR = 5e-1;
    private static final double VELOCITY_ERROR = 5e-2;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    private static final double PROPAGATION_ERROR = 1.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private int mUpdateGNSSMeasurementsStart;
    private int mUpdateGNSSMeasurementsEnd;
    private int mUpdateBodyKinematicsStart;
    private int mUpdateBodyKinematicsEnd;
    private int mPropagateStart;
    private int mPropagateEnd;
    private int mReset;

    @Test
    public void testConstructor()
            throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        INSGNSSLooselyCoupledKalmanFilteredEstimator estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final Time epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        Time lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 2
        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime2);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime2);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        INSLooselyCoupledKalmanConfig kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 3
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime3);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime3);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval));

        // test constructor 4
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime4);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime4);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 5
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime5 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime5);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime5);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval));

        // test constructor 6
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime6 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime6);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime6);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 7
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime7 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime7);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime7);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, this));

        // test constructor 8
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime8 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime8);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime8);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, this));

        // test constructor 9
        final var epochIntervalTime9 = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime9);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime10 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime10);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime10);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 10
        epochIntervalTime.setValue(epochInterval);
        epochIntervalTime.setUnit(TimeUnit.SECOND);
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime11 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime11);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime11);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final Time wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime));

        // test constructor 11
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime12 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime12);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime12);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, this));

        // test constructor 12
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime13 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime13);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime13);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, this));

        // test constructor 13
        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime14 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime14);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime14);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        CoordinateTransformation c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 14
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime15 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime15);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime15);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 15
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime16 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime16);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime16);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, c));

        // test constructor 16
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime17 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime17);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime17);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 17
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime18 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime18);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime18);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c));

        // test constructor 18
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime19 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime19);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime19);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 19
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime20 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime20);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime20);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, c, this));

        // test constructor 20
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime21 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime21);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime21);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, c, this));

        // test constructor 21
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime22 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime22);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime22);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c));

        // test constructor 22
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime23 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime23);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime23);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c));

        // test constructor 23
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime24 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime24);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime24);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, c, this));

        // test constructor 24
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, c, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime25 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime25);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime25);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, c, this));

        // test constructor 25
        final INSLooselyCoupledKalmanInitializerConfig initialConfig =
                generateInitConfig();
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime26 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime26);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime26);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        INSLooselyCoupledKalmanInitializerConfig initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 26
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime27 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime27);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime27);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 27
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime28 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime28);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime28);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig));

        // test constructor 28
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime29 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime29);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime29);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 29
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime30 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime30);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime30);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig));

        // test constructor 30
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime31 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime31);
        assertEquals(epochIntervalTime31, new Time(0.0, TimeUnit.SECOND));
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 31
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime32 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime32);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime32);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, this));

        // test constructor 32
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime33 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime33);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime33);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, this));

        // test constructor 33
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime34 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime34);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime34);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig));

        // test constructor 34
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime35 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime35);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime35);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig));

        // test constructor 35
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime36 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime36);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime36);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 36
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        final var epochIntervalTime37 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime37);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime37);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, this));

        // test constructor 37
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime38 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime38);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime38);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 38
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime39 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime39);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime39);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // test constructor 39
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime40 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime40);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime40);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c));

        // test constructor 40
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime41 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime41);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime41);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 41
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime42 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime42);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime42);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c));

        // test constructor 42
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime43 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime43);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime43);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // test constructor 43
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime44 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime44);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime44);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, c, this));

        // test constructor 44
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime45 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime45);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime45);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, -epochInterval, initialConfig, c, this));

        // test constructor 45
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime46 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime46);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime46);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c));

        // test constructor 46
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime47 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime47);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime47);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME)));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c));

        // test constructor 47
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, c, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime48 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime48);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime48);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, c, this));

        // test constructor 48
        estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, c,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        final var epochIntervalTime49 = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime49);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime49);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(c, estimator.getCoordinateTransformation());
        c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        assertEquals(c, c2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME), this));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, wrongEpochIntervalTime, initialConfig, c, this));
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetEpochInterval() throws LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        // set new value
        estimator.setEpochInterval(1.0);

        // check
        assertEquals(1.0, estimator.getEpochInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setEpochInterval(-1.0));
    }

    @Test
    public void testGetSetEpochIntervalAsTime() throws LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        final Time epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(0.0, epochInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, epochInterval1.getUnit());

        // set new value
        final Time epochInterval2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setEpochInterval(epochInterval2);

        // check
        final Time epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochInterval3);
        final Time epochInterval4 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);
    }

    @Test
    public void testGetSetConfig() throws LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getConfig());
        assertFalse(estimator.getConfig(null));

        // set new value
        final INSLooselyCoupledKalmanConfig config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final INSLooselyCoupledKalmanConfig config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    public void testGetSetCoordinateTransformation() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getCoordinateTransformation());
        assertFalse(estimator.getCoordinateTransformation(null));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        estimator.setCoordinateTransformation(c1);

        // check
        final CoordinateTransformation c2 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(estimator.getCoordinateTransformation(c2));
        final CoordinateTransformation c3 = estimator.getCoordinateTransformation();

        assertEquals(c1, c2);
        assertEquals(c1, c3);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setCoordinateTransformation(new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)));
    }

    @Test
    public void testGetSetInitialConfig() throws LockedException {
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSGNSSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getInitialConfig());
        assertFalse(estimator.getInitialConfig(null));

        // set new value
        final INSLooselyCoupledKalmanInitializerConfig config1 = generateInitConfig();
        estimator.setInitialConfig(config1);

        // check
        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(config2));
        final INSLooselyCoupledKalmanInitializerConfig config3 = estimator.getInitialConfig();

        assertEquals(config1, config2);
        assertSame(config1, config3);
    }

    @Test
    public void testIsUpdateMeasurementsReady() {
        assertFalse(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(null));

        final List<GNSSMeasurement> measurements = new ArrayList<>();
        assertFalse(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));

        for (int i = 0; i < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS; i++) {
            measurements.add(new GNSSMeasurement());
        }
        assertTrue(INSGNSSLooselyCoupledKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));
    }

    @Test
    public void testUpdateMeasurementsWithoutKinematicsAndWithoutInitialAttitude() throws LockedException,
            NotReadyException, INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final com.irurueta.navigation.frames.NEDPosition nedUserPosition =
                    new com.irurueta.navigation.frames.NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final com.irurueta.navigation.frames.NEDPosition nedSatPosition =
                        new com.irurueta.navigation.frames.NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
            assertNull(estimator.getKinematics());
            assertNull(estimator.getCorrectedKinematics());
            assertEquals(new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                    estimator.getCoordinateTransformation());
            final CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(c2, new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(1, mUpdateBodyKinematicsStart);
            assertEquals(1, mUpdateBodyKinematicsEnd);
            assertEquals(1, mPropagateStart);
            assertEquals(1, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateMeasurementsWithBodyKinematicsAndWithoutInitialAttitude() throws LockedException,
            INSGNSSException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            final BodyKinematics kinematics = new BodyKinematics();
            assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertEquals(kinematics, estimator.getKinematics());
            BodyKinematics kinematics2 = new BodyKinematics();
            assertTrue(estimator.getKinematics(kinematics2));
            assertEquals(kinematics, kinematics2);
            assertNotNull(estimator.getCorrectedKinematics());
            assertTrue(estimator.getCorrectedKinematics(new BodyKinematics()));
            assertEquals(estimator.getCoordinateTransformation(), new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));
            CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(c2, new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, 2.0 * timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());
            assertEquals(new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                    estimator.getCoordinateTransformation());
            c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME),
                    c2);

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateMeasurementsWithoutBodyKinematicsAndWithInitialAttitude()
            throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, c, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertEquals(c, estimator.getCoordinateTransformation());
            CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(c2, c);

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
            assertNull(estimator.getKinematics());
            assertNull(estimator.getCorrectedKinematics());
            assertTrue(estimator.getCoordinateTransformation().equals(c, ABSOLUTE_ERROR));
            c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertTrue(c.equals(c2, ABSOLUTE_ERROR));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateMeasurementsWithKinematicsAndInitialAttitude() throws LockedException, INSGNSSException,
            InvalidSourceAndDestinationFrameTypeException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, c, this);

            final BodyKinematics kinematics = new BodyKinematics();
            assertTrue(estimator.updateBodyKinematics(kinematics, timeSeconds));

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertEquals(kinematics, estimator.getKinematics());
            BodyKinematics kinematics2 = new BodyKinematics();
            assertTrue(estimator.getKinematics(kinematics2));
            assertEquals(kinematics, kinematics2);
            assertNotNull(estimator.getCorrectedKinematics());
            assertTrue(estimator.getCorrectedKinematics(new BodyKinematics()));
            assertTrue(estimator.getCoordinateTransformation().equals(c, ABSOLUTE_ERROR));
            CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertTrue(c2.equals(c, ABSOLUTE_ERROR));

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, 2.0 * timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());
            assertTrue(estimator.getCoordinateTransformation().equals(c, ABSOLUTE_ERROR));
            c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertTrue(c.equals(c2, ABSOLUTE_ERROR));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test(expected = NotReadyException.class)
    public void testUpdateMeasurementsWhenNotReadyThrowsNotReadyException() throws LockedException, NotReadyException,
            INSGNSSException {

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, initConfig);

        estimator.updateMeasurements(Collections.<GNSSMeasurement>emptyList(), 0.0);
    }

    @Test
    public void testUpdateKinematicsWithZeroSpecificForceAndAngularRate() throws LockedException, NotReadyException,
            INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            final BodyKinematics kinematics = new BodyKinematics();
            assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());
            assertEquals(estimator.getCoordinateTransformation(), new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));
            CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(c2, new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(2, mUpdateBodyKinematicsStart);
            assertEquals(2, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateKinematicsWithFreeFallSpecificForceAndZeroAngularRate() throws LockedException,
            NotReadyException, INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefUserPosition.getX(),
                    ecefUserPosition.getY(), ecefUserPosition.getZ());

            // because there is no attitude, the specific force is directly the
            // gravity.
            final BodyKinematics kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    0.0, 0.0, 0.0);
            assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(2, mUpdateBodyKinematicsStart);
            assertEquals(2, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateKinematicsWithZeroSpecificForceAndRotationOnly() throws LockedException, INSGNSSException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(new BodyKinematics()));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            final double angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final double angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final double angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));

            final BodyKinematics kinematics = new BodyKinematics(0.0, 0.0, 0.0,
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(), new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(2, mUpdateBodyKinematicsStart);
            assertEquals(2, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateKinematicsWithFreeFallSpecificForceAndRotation() throws LockedException, NotReadyException,
            INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                    ecefUserPosition.getX(), ecefUserPosition.getY(), ecefUserPosition.getZ());

            final double angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final double angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final double angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));

            // because there is no attitude, the specific force is directly the
            // gravity.
            final BodyKinematics kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.updateBodyKinematics(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.updateBodyKinematics(kinematics, new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(2, mUpdateBodyKinematicsStart);
            assertEquals(2, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testPropagate() throws LockedException, NotReadyException, INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            final ECEFPosition estimatedPosition1 = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = estimation1.getEcefVelocity();

            final double diffX1 = Math.abs(ecefUserPosition.getX() - estimatedPosition1.getX());
            final double diffY1 = Math.abs(ecefUserPosition.getY() - estimatedPosition1.getY());
            final double diffZ1 = Math.abs(ecefUserPosition.getZ() - estimatedPosition1.getZ());
            final double posError1 = Math.max(diffX1, Math.max(diffY1, diffZ1));
            if (posError1 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition1, POSITION_ERROR));

            final double diffVx1 = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity1.getVx());
            final double diffVy1 = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity1.getVy());
            final double diffVz1 = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity1.getVz());
            final double velError = Math.max(diffVx1, Math.max(diffVy1, diffVz1));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity1, VELOCITY_ERROR));

            // propagate
            assertTrue(estimator.propagate(2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            final ECEFPosition estimatedPosition3 = estimation3.getEcefPosition();
            final ECEFVelocity estimatedVelocity3 = estimation3.getEcefVelocity();

            final double diffX3 = Math.abs(ecefUserPosition.getX() - estimatedPosition3.getX());
            final double diffY3 = Math.abs(ecefUserPosition.getY() - estimatedPosition3.getY());
            final double diffZ3 = Math.abs(ecefUserPosition.getZ() - estimatedPosition3.getZ());
            final double posError3 = Math.max(diffX3, Math.max(diffY3, diffZ3));
            if (posError3 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition3, PROPAGATION_ERROR));

            final double diffVx3 = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity3.getVx());
            final double diffVy3 = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity3.getVy());
            final double diffVz3 = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity3.getVz());
            final double velError3 = Math.max(diffVx3, Math.max(diffVy3, diffVz3));
            if (velError3 > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity3, PROPAGATION_ERROR));

            final Matrix covariance1 = state1.getCovariance();
            final Matrix covariance3 = state3.getCovariance();

            final double norm1 = Utils.normF(covariance1);
            final double norm3 = Utils.normF(covariance3);
            assertTrue(norm3 >= norm1);

            assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation4 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state4 = estimator.getState();

            assertEquals(estimation3, estimation4);
            assertEquals(state3, state4);

            assertEquals(1, mUpdateGNSSMeasurementsStart);
            assertEquals(1, mUpdateGNSSMeasurementsEnd);
            assertEquals(1, mUpdateBodyKinematicsStart);
            assertEquals(1, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testPropagateWhenNotReadyReturnsFalse() throws LockedException, INSGNSSException {
        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

        final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator = new INSGNSSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, initConfig);

        assertFalse(estimator.propagate(0.0));
    }

    @Test
    public void testReset() throws LockedException, NotReadyException, INSGNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition,
                    ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final int numSatellites = config.getNumberOfSatellites();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities = new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final com.irurueta.navigation.frames.NEDPosition nedSatPosition = new NEDPosition(satLatitude,
                        satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition,
                        ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator.generate(timeSeconds,
                    satellitePositionsAndVelocities, ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();
            final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSGNSSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, this);

            reset();
            assertEquals(0, mUpdateGNSSMeasurementsStart);
            assertEquals(0, mUpdateGNSSMeasurementsEnd);
            assertEquals(0, mUpdateBodyKinematicsStart);
            assertEquals(0, mUpdateBodyKinematicsEnd);
            assertEquals(0, mPropagateStart);
            assertEquals(0, mPropagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertNull(estimator.getCoordinateTransformation());
            assertFalse(estimator.getCoordinateTransformation(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final INSGNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertNull(estimator.getKinematics());
            assertNull(estimator.getCorrectedKinematics());
            assertEquals(estimator.getCoordinateTransformation(), new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));
            final CoordinateTransformation c2 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            assertTrue(estimator.getCoordinateTransformation(c2));
            assertEquals(c2, new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            // reset
            assertEquals(0, mReset);

            estimator.reset();

            assertEquals(1, mReset);
            assertNull(estimator.getMeasurements());
            assertNull(estimator.getEstimation());
            assertNull(estimator.getState());
            assertNull(estimator.getLastStateTimestamp());
            assertNull(estimator.getCoordinateTransformation());
            assertNull(estimator.getKinematics());
            assertFalse(estimator.isRunning());

            // update again with same timestamp now it does make an action
            assertTrue(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final GNSSEstimation estimation2 = estimator.getEstimation();
            final INSLooselyCoupledKalmanState state2 = estimator.getState();

            assertEquals(estimation1, estimation2);
            assertEquals(state1, state2);

            assertEquals(2, mUpdateGNSSMeasurementsStart);
            assertEquals(2, mUpdateGNSSMeasurementsEnd);
            assertEquals(2, mUpdateBodyKinematicsStart);
            assertEquals(2, mUpdateBodyKinematicsEnd);
            assertEquals(2, mPropagateStart);
            assertEquals(2, mPropagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onUpdateGNSSMeasurementsStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateGNSSMeasurementsStart++;
    }

    @Override
    public void onUpdateGNSSMeasurementsEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateGNSSMeasurementsEnd++;
    }

    @Override
    public void onUpdateBodyKinematicsStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateBodyKinematicsStart++;
    }

    @Override
    public void onUpdateBodyKinematicsEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateBodyKinematicsEnd++;
    }

    @Override
    public void onPropagateStart(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateStart++;
    }

    @Override
    public void onPropagateEnd(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateEnd++;
    }

    @Override
    public void onReset(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void reset() {
        mUpdateGNSSMeasurementsStart = 0;
        mUpdateGNSSMeasurementsEnd = 0;
        mUpdateBodyKinematicsStart = 0;
        mUpdateBodyKinematicsEnd = 0;
        mPropagateStart = 0;
        mPropagateEnd = 0;
        mReset = 0;
    }

    private static void checkLocked(final INSGNSSLooselyCoupledKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setConfig(null));
        assertThrows(LockedException.class, () -> estimator.setCoordinateTransformation(null));
        assertThrows(LockedException.class, () -> estimator.setInitialConfig(null));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.updateBodyKinematics(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.updateBodyKinematics(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.propagate(0.0));
        assertThrows(LockedException.class, () -> estimator.propagate(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
    }

    private static INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                positionNoiseSD, velocityNoiseSD);
    }

    private static GNSSConfig generateConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_MASK_ANGLE_DEGREES, MAX_MASK_ANGLE_DEGREES);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
    }
}
