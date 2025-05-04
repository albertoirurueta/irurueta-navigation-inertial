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

import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class INSLooselyCoupledKalmanFilteredEstimatorTest implements INSLooselyCoupledKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double POSITION_ERROR = 5.0;
    private static final double VELOCITY_ERROR = 5.0;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private int updateStart;
    private int updateEnd;
    private int propagateStart;
    private int propagateEnd;
    private int reset;

    @Test
    void testConstructor() throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        var epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        var lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 2
        final var kalmanConfig = generateKalmanConfig();
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        var kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 3
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval));

        // test constructor 4
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 5
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval));

        // test constructor 6
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 7
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, this));

        // test constructor 8
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, this));

        // test constructor 9
        epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 10
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final var wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime));

        // test constructor 11
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, this));

        // test constructor 12
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, this));

        // test constructor 13
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
        final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

        final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

        final var ecefUserPosition = new ECEFPosition();
        final var ecefUserVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                ecefUserVelocity);

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        var frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 14
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 15
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                frame));

        // test constructor 16
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 17
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, frame));

        // test constructor 18
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 19
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                frame, this));

        // test constructor 20
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, frame, this));

        // test constructor 21
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, frame));

        // test constructor 22
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, frame));

        // test constructor 23
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, frame, this));

        // test constructor 24
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, frame, this));

        // test constructor 25
        final var initialConfig = generateInitConfig();
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        INSLooselyCoupledKalmanInitializerConfig initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 26
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 27
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                initialConfig));

        // test constructor 28
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 29
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig));

        // test constructor 30
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 31
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, this));

        // test constructor 32
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, 
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, this));

        // test constructor 33
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig));

        // test constructor 34
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig));

        // test constructor 35
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 36
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, this));

        // test constructor 37
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 38
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 39
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                initialConfig, frame));

        // test constructor 40
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor 41
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, frame));

        // test constructor 42
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // test constructor 43
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                -epochInterval, initialConfig, frame, this));

        // test constructor 44
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochInterval, initialConfig, frame,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                -epochInterval, initialConfig, frame, this));

        // test constructor 45
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, frame));

        // test constructor 46
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, frame));

        // test constructor 47
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime, initialConfig, frame, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(
                wrongEpochIntervalTime, initialConfig, frame, this));

        // test constructor 48
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, epochIntervalTime, initialConfig, frame,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(frame, estimator.getFrame());
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                wrongEpochIntervalTime, initialConfig, frame, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetEpochInterval() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        // set a new value
        estimator.setEpochInterval(1.0);

        // check
        assertEquals(1.0, estimator.getEpochInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setEpochInterval(-1.0));
    }

    @Test
    void testGetSetEpochIntervalAsTime() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        final var epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(0.0, epochInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, epochInterval1.getUnit());

        // set a new value
        final var epochInterval2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setEpochInterval(epochInterval2);

        // check
        final var epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochInterval3);
        final var epochInterval4 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);
    }

    @Test
    void testGetSetConfig() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getConfig());
        assertFalse(estimator.getConfig(null));

        // set a new value
        final var config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final var config2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final var config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    void testGetSetFrame() throws LockedException, InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
        final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

        final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

        final var ecefUserPosition = new ECEFPosition();
        final var ecefUserVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                ecefUserVelocity);

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));

        // set a new value
        estimator.setFrame(frame);

        // check
        assertEquals(frame, estimator.getFrame());

        final var frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
    }

    @Test
    void testGetSetInitialConfig() throws LockedException {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getInitialConfig());
        assertFalse(estimator.getInitialConfig(null));

        // set a new value
        final var config1 = generateInitConfig();
        estimator.setInitialConfig(config1);

        // check
        final var config2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(config2));
        final var config3 = estimator.getInitialConfig();

        assertEquals(config1, config2);
        assertSame(config1, config3);
    }

    @Test
    void testUpdateWithZeroSpecificForceAndAngularRate() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InvalidRotationMatrixException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

            final var kalmanConfig = generateKalmanConfig();
            final var initConfig = generateInitConfig();
            final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame, 
                    this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final var kinematics = new BodyKinematics();
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(state1.getFrame(), estimator.getFrame());

            final var estimatedPosition1 = state1.getEcefPosition();
            final var estimatedVelocity1 = state1.getEcefVelocity();
            final var estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with the same timestamp makes no action
            assertFalse(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with a new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state4 = estimator.getState();
            final var estimatedPosition4 = state4.getEcefPosition();
            final var estimatedVelocity4 = state4.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition4.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition4.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition4.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity4.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity4.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity4.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(2, updateStart);
            assertEquals(2, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testUpdateWithSpecificForceAndZeroAngularRate() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, INSException, InvalidRotationMatrixException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

            final var kalmanConfig = generateKalmanConfig();
            final var initConfig = generateInitConfig();
            final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame,
                    this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                    ecefUserPosition.getX(), ecefUserPosition.getY(), ecefUserPosition.getZ());

            final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    0.0, 0.0, 0.0);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimator.getFrame(), state1.getFrame());

            final var estimatedPosition1 = state1.getEcefPosition();
            final var estimatedVelocity1 = state1.getEcefVelocity();
            final var estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with the same timestamp makes no action
            assertFalse(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with a new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state4 = estimator.getState();
            final var estimatedPosition4 = state4.getEcefPosition();
            final var estimatedVelocity4 = state4.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition4.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition4.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition4.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity4.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity4.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity4.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(2, updateStart);
            assertEquals(2, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testUpdateWithZeroSpecificForceAndRotationOnly() throws LockedException, NotReadyException, INSException,
            InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

            final var kalmanConfig = generateKalmanConfig();
            final var initConfig = generateInitConfig();
            final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame,
                    this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, 
                    MAX_DEGREES_PER_SECOND));
            final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));

            final var kinematics = new BodyKinematics(0.0, 0.0, 0.0, 
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(state1.getFrame(), estimator.getFrame());

            final var estimatedPosition1 = state1.getEcefPosition();
            final var estimatedVelocity1 = state1.getEcefVelocity();
            final var estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with the same timestamp makes no action
            assertFalse(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with a new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state4 = estimator.getState();
            final var estimatedPosition4 = state4.getEcefPosition();
            final var estimatedVelocity4 = state4.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition4.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition4.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition4.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity4.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity4.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity4.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(2, updateStart);
            assertEquals(2, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testUpdateWithSpecificForceAndRotation() throws LockedException, NotReadyException, INSException,
            InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

            final var kalmanConfig = generateKalmanConfig();
            final var initConfig = generateInitConfig();
            final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame,
                    this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefUserPosition.getX(),
                    ecefUserPosition.getY(), ecefUserPosition.getZ());

            final var angularRateX = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND, 
                    MAX_DEGREES_PER_SECOND));
            final var angularRateY = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));
            final var angularRateZ = Math.toRadians(randomizer.nextDouble(MIN_DEGREES_PER_SECOND,
                    MAX_DEGREES_PER_SECOND));

            final var kinematics = new BodyKinematics(gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(state1.getFrame(), estimator.getFrame());

            final var estimatedPosition1 = state1.getEcefPosition();
            final var estimatedVelocity1 = state1.getEcefVelocity();
            final var estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with the same timestamp makes no action
            assertFalse(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with a new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final var state4 = estimator.getState();
            final var estimatedPosition4 = state4.getEcefPosition();
            final var estimatedVelocity4 = state4.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition4.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition4.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition4.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity4.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity4.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity4.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(2, updateStart);
            assertEquals(2, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testUpdateWhenNotReady() {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isUpdateReady());

        assertThrows(NotReadyException.class, () -> estimator.update(new BodyKinematics(), 0.0));
    }

    @Test
    void testPropagate() throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();

        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

        final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
        final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

        final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

        final var ecefUserPosition = new ECEFPosition();
        final var ecefUserVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                ecefUserVelocity);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame, 
                this);

        reset();
        assertEquals(0, updateStart);
        assertEquals(0, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // update kinematics for the first time, makes no change
        final var kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final var state1 = estimator.getState();
        assertNotNull(state1);

        final var state2 = new INSLooselyCoupledKalmanState();
        assertTrue(estimator.getState(state2));

        assertEquals(state1, state2);

        assertEquals(estimator.getFrame(), state1.getFrame());

        final var estimatedPosition1 = state1.getEcefPosition();
        final var estimatedVelocity1 = state1.getEcefVelocity();
        final var estimatedC1 = state1.getC();

        assertEquals(estimatedPosition1, ecefUserPosition);
        assertEquals(estimatedVelocity1, ecefUserVelocity);
        assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

        // propagate
        assertTrue(estimator.propagate(2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(2.0 * timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final var state3 = estimator.getState();
        final var estimatedPosition3 = state3.getEcefPosition();
        final var estimatedVelocity3 = state3.getEcefVelocity();
        final var estimatedC3 = state3.getC();

        assertEquals(estimatedPosition3, ecefUserPosition);
        assertEquals(estimatedVelocity3, ecefUserVelocity);
        assertTrue(estimatedC3.equals(c, ABSOLUTE_ERROR));
        // state is not equal because covariance has changed
        assertNotEquals(state1, state3);

        final var covariance1 = state1.getCovariance();
        final var covariance3 = state3.getCovariance();

        final var norm1 = Utils.normF(covariance1);
        final var norm3 = Utils.normF(covariance3);
        assertTrue(norm3 >= norm1);

        // propagate again with the same timestamp has no effect
        assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

        final var state4 = estimator.getState();
        final var estimatedPosition4 = state4.getEcefPosition();
        final var estimatedVelocity4 = state4.getEcefVelocity();
        final var estimatedC4 = state4.getC();

        assertEquals(estimatedPosition4, ecefUserPosition);
        assertEquals(estimatedVelocity4, ecefUserVelocity);
        assertTrue(estimatedC4.equals(c, ABSOLUTE_ERROR));
        assertEquals(state3, state4);

        assertEquals(1, updateStart);
        assertEquals(1, updateEnd);
        assertEquals(2, propagateStart);
        assertEquals(2, propagateEnd);
    }

    @Test
    void testPropagateWhenNotReady() {
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isPropagateReady());

        assertThrows(NotReadyException.class, () -> estimator.propagate(0.0));
    }

    @Test
    void testReset() throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            INSException, InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();

        final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

        final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
        final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

        final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
        final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

        final var ecefUserPosition = new ECEFPosition();
        final var ecefUserVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                ecefUserVelocity);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();
        final var estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame,
                this);

        reset();
        assertEquals(0, updateStart);
        assertEquals(0, updateEnd);
        assertEquals(0, propagateStart);
        assertEquals(0, propagateEnd);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // update kinematics for the first time, makes no change
        final var kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
        final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(new Time(timeSeconds, TimeUnit.SECOND), timestamp);
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final var state1 = estimator.getState();
        assertNotNull(state1);

        assertEquals(estimator.getFrame(), state1.getFrame());

        final var estimatedPosition1 = state1.getEcefPosition();
        final var estimatedVelocity1 = state1.getEcefVelocity();
        final var estimatedC1 = state1.getC();

        assertEquals(estimatedPosition1, ecefUserPosition);
        assertEquals(estimatedVelocity1, ecefUserVelocity);
        assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

        // reset
        assertEquals(0, reset);

        estimator.reset();

        assertEquals(1, reset);
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getFrame());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.isRunning());

        assertFalse(estimator.isUpdateReady());

        // set frame
        estimator.setFrame(frame);

        assertTrue(estimator.isUpdateReady());

        // update again with the same timestamp now it does make an action
        assertTrue(estimator.update(kinematics, new Time(timeSeconds, TimeUnit.SECOND)));

        assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
        final var state2 = estimator.getState();

        assertTrue(state1.equals(state2, ABSOLUTE_ERROR));

        assertEquals(2, updateStart);
        assertEquals(2, updateEnd);
        assertEquals(2, propagateStart);
        assertEquals(2, propagateEnd);
    }

    @Override
    public void onUpdateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateStart++;
    }

    @Override
    public void onUpdateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateEnd++;
    }

    @Override
    public void onPropagateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateStart++;
    }

    @Override
    public void onPropagateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateEnd++;
    }

    @Override
    public void onReset(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        updateStart = 0;
        updateEnd = 0;
        propagateStart = 0;
        propagateEnd = 0;
        reset = 0;
    }

    private static void checkLocked(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setConfig(null));
        assertThrows(LockedException.class, () -> estimator.setFrame(null));
        assertThrows(LockedException.class, () -> estimator.setInitialConfig(null));
        assertThrows(LockedException.class, () -> estimator.update(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.update(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.propagate(0.0));
        assertThrows(LockedException.class, () -> estimator.propagate(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
    }

    private static INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                positionNoiseSD, velocityNoiseSD);
    }
}
