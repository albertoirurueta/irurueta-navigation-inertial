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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class NEDKinematicsEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_VARIATION_DEGREES = -5.0;
    private static final double MAX_ANGLE_VARIATION_DEGREES = 5.0;

    private static final double MIN_POSITION_VARIATION_DEGREES = -1e-4;
    private static final double MAX_POSITION_VARIATION_DEGREES = 1e-4;

    private static final double MIN_HEIGHT_VARIATION = -0.5;
    private static final double MAX_HEIGHT_VARIATION = 0.5;

    private static final double MIN_VELOCITY_VARIATION = -0.1;
    private static final double MAX_VELOCITY_VARIATION = 0.1;

    private static final double SCALING_THRESHOLD = 2e-5;
    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 100;

    @Test
    void testEstimate() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var longitude = frame.getLongitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldLongitude = oldFrame.getLongitude();
        final var oldHeight = oldFrame.getHeight();

        final var estimator = new NEDKinematicsEstimator();

        final var k1 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, 
                oldLatitude, oldHeight, k1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, 
                oldLatitude, oldHeight, k2);

        final var velocity = new NEDVelocity(vn, ve, vd);
        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var k3 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, latitude, height, 
                oldLatitude, oldHeight, k3);

        final var k4 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, latitude, height, oldLatitude, oldHeight, k4);

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var k5 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k5);

        final var k6 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k6);

        final var position = new NEDPosition(latitude, longitude, height);
        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var k7 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, position, oldPosition, k7);

        final var k8 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, position, oldPosition, k8);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var k9 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                position, oldPosition, k9);

        final var k10 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, position,
                oldPosition, k10);

        final var k11 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, position, oldPosition, k11);

        final var k12 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity, position, oldPosition, k12);

        final var k13 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd, oldLatitude, oldHeight, k13);

        final var k14 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd, oldLatitude, oldHeight, k14);

        final var k15 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude, oldHeight, k15);

        final var k16 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight, k16);

        final var k17 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitudeAngle, oldHeightDistance, k17);

        final var k18 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldLatitudeAngle, oldHeightDistance, k18);

        final var k19 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude, oldHeight, k19);

        final var k20 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight, k20);

        final var k21 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd, oldPosition, k21);

        final var k22 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd, oldPosition, k22);

        final var k23 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN, oldSpeedE, oldSpeedD, oldPosition, k23);

        final var k24 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldSpeedN, oldSpeedE, oldSpeedD, oldPosition, k24);

        final var k25 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldPosition, k25);

        final var k26 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldPosition, k26);

        final var k27 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height, oldFrame, k27);

        final var k28 = new BodyKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, latitude, height, oldFrame, k28);

        final var k29 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, latitude, height, oldFrame, k29);

        final var k30 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, latitude, height, oldFrame, k30);

        final var k31 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle, heightDistance, oldFrame, k31);

        final var k32 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, latitudeAngle, heightDistance, oldFrame, k32);

        final var k33 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame, k33);

        final var k34 = new BodyKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, position, oldFrame, k34);

        final var k35 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD, position, oldFrame, k35);

        final var k36 = new BodyKinematics();
        estimator.estimate(timeInterval, c, speedN, speedE, speedD, position, oldFrame, k36);

        final var k37 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame, k37);

        final var k38 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, position, oldFrame, k38);

        final var k39 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldFrame, k39);

        final var k40 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldFrame, k40);

        final var k41 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                latitude, height, oldLatitude, oldHeight, k41);

        final var k42 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, latitude,
                height, oldLatitude, oldHeight, k42);

        final var k43 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitudeAngle, height,
                oldLatitudeAngle, oldHeight, k43);

        final var k44 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitudeAngle, height,
                oldLatitudeAngle, oldHeight, k44);

        final var k45 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k45);

        final var k46 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k46);

        final var k47 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance, k47);

        final var k48 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance, k48);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
        assertEquals(k1, k27);
        assertEquals(k1, k28);
        assertEquals(k1, k29);
        assertEquals(k1, k30);
        assertEquals(k1, k31);
        assertEquals(k1, k32);
        assertEquals(k1, k33);
        assertEquals(k1, k34);
        assertEquals(k1, k35);
        assertEquals(k1, k36);
        assertEquals(k1, k37);
        assertEquals(k1, k38);
        assertEquals(k1, k39);
        assertEquals(k1, k40);
        assertEquals(k1, k41);
        assertEquals(k1, k42);
        assertEquals(k1, k43);
        assertEquals(k1, k44);
        assertEquals(k1, k45);
        assertEquals(k1, k46);
        assertEquals(k1, k47);
        assertEquals(k1, k48);
    }

    @Test
    void testEstimateAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var longitude = frame.getLongitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldLongitude = oldFrame.getLongitude();
        final var oldHeight = oldFrame.getHeight();

        final var estimator = new NEDKinematicsEstimator();

        final var k1 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var k2 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight);

        final var velocity = new NEDVelocity(vn, ve, vd);
        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var k3 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight);

        final var k4 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, latitude, height,
                oldLatitude, oldHeight);

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var k5 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        final var k6 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        final var position = new NEDPosition(latitude, longitude, height);
        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var k7 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition);

        final var k8 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var k9 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

        final var k10 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

        final var k11 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, position,
                oldPosition);

        final var k12 = estimator.estimateAndReturnNew(timeInterval, c, oldC, velocity, oldVelocity, position,
                oldPosition);

        final var k13 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final var k14 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final var k15 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);

        final var k16 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight);

        final var k17 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldLatitudeAngle, oldHeightDistance);

        final var k18 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVelocity,
                oldLatitudeAngle, oldHeightDistance);

        final var k19 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);

        final var k20 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight);

        final var k21 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition);

        final var k22 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVn, oldVe, oldVd, oldPosition);

        final var k23 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, oldPosition);

        final var k24 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                oldPosition);

        final var k25 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldPosition);

        final var k26 = estimator.estimateAndReturnNew(timeInterval, frame, oldC, oldVelocity, oldPosition);

        final var k27 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame);

        final var k28 = estimator.estimateAndReturnNew(timeInterval, c, vn, ve, vd, latitude, height, oldFrame);

        final var k29 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity, latitude, height, oldFrame);

        final var k30 = estimator.estimateAndReturnNew(timeInterval, c, velocity, latitude, height, oldFrame);

        final var k31 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle,
                heightDistance, oldFrame);

        final var k32 = estimator.estimateAndReturnNew(timeInterval, c, velocity, latitudeAngle, heightDistance,
                oldFrame);

        final var k33 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame);

        final var k34 = estimator.estimateAndReturnNew(timeInterval, c, vn, ve, vd, position, oldFrame);

        final var k35 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD, position,
                oldFrame);

        final var k36 = estimator.estimateAndReturnNew(timeInterval, c, speedN, speedE, speedD, position, oldFrame);

        final var k37 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame);

        final var k38 = estimator.estimateAndReturnNew(timeInterval, c, velocity, position, oldFrame);

        final var k39 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, frame, oldFrame);

        final var k40 = estimator.estimateAndReturnNew(timeInterval, frame, oldFrame);

        final var k41 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight);

        final var k42 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight);

        final var k43 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final var k44 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final var k45 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, heightDistance, oldLatitude, oldHeightDistance);

        final var k46 = estimator.estimateAndReturnNew(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, heightDistance, oldLatitude, oldHeightDistance);

        final var k47 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        final var k48 = estimator.estimateAndReturnNew(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
        assertEquals(k1, k27);
        assertEquals(k1, k28);
        assertEquals(k1, k29);
        assertEquals(k1, k30);
        assertEquals(k1, k31);
        assertEquals(k1, k32);
        assertEquals(k1, k33);
        assertEquals(k1, k34);
        assertEquals(k1, k35);
        assertEquals(k1, k36);
        assertEquals(k1, k37);
        assertEquals(k1, k38);
        assertEquals(k1, k39);
        assertEquals(k1, k40);
        assertEquals(k1, k41);
        assertEquals(k1, k42);
        assertEquals(k1, k43);
        assertEquals(k1, k44);
        assertEquals(k1, k45);
        assertEquals(k1, k46);
        assertEquals(k1, k47);
        assertEquals(k1, k48);
    }

    @Test
    void testEstimateKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, RankDeficientMatrixException, DecomposerException {

        for (var t = 0; t < TIMES; t++) {
            final var oldFrame = createOldNedFrame();

            final var frame = createNewNedFrame(oldFrame);

            final var c = frame.getCoordinateTransformation();
            final var oldC = oldFrame.getCoordinateTransformation();

            final var vn = frame.getVn();
            final var ve = frame.getVe();
            final var vd = frame.getVd();

            final var oldVn = oldFrame.getVn();
            final var oldVe = oldFrame.getVe();
            final var oldVd = oldFrame.getVd();

            final var latitude = frame.getLatitude();
            final var longitude = frame.getLongitude();
            final var height = frame.getHeight();

            final var oldLatitude = oldFrame.getLatitude();
            final var oldLongitude = oldFrame.getLongitude();
            final var oldHeight = oldFrame.getHeight();

            final var k1 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, height, oldLatitude, oldHeight, k1);

            final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final var k2 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, height, oldLatitude, oldHeight, k2);

            final var velocity = new NEDVelocity(vn, ve, vd);
            final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
            final var k3 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    latitude, height, oldLatitude, oldHeight, k3);

            final var k4 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, latitude, height,
                    oldLatitude, oldHeight, k4);

            final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
            final var heightDistance = new Distance(height, DistanceUnit.METER);
            final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
            final var k5 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance, k5);

            final var k6 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, latitudeAngle,
                    heightDistance, oldLatitudeAngle, oldHeightDistance, k6);

            final var position = new NEDPosition(latitude, longitude, height);
            final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
            final var k7 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    position, oldPosition, k7);

            final var k8 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    position, oldPosition, k8);

            final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
            final var k9 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k9);

            final var k10 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k10);

            final var k11 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity, position,
                    oldPosition, k11);

            final var k12 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, velocity, oldVelocity, position,
                    oldPosition, k12);

            final var k13 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight, k13);

            final var k14 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight, k14);

            final var k15 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight, k15);

            final var k16 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight,
                    k16);

            final var k17 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight, k17);

            final var k18 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldLatitude, oldHeight,
                    k18);

            final var k19 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldPosition, k19);

            final var k20 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldVn, oldVe, oldVd, oldPosition, k20);

            final var k21 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN, oldSpeedE,
                    oldSpeedD, oldPosition, k21);

            final var k22 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                    oldPosition, k22);

            final var k23 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldPosition,
                    k23);

            final var k24 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC, oldVelocity, oldPosition, k24);

            final var k25 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height, oldFrame,
                    k25);

            final var k26 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, vn, ve, vd, latitude, height, oldFrame, k26);

            final var k27 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, velocity, latitude, height, oldFrame,
                    k27);

            final var k28 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, velocity, latitude, height, oldFrame, k28);

            final var k29 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle, heightDistance,
                    oldFrame, k29);

            final var k30 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, velocity, latitudeAngle, heightDistance,
                    oldFrame, k30);

            final var k31 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame, k31);

            final var k32 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, vn, ve, vd, position, oldFrame, k32);

            final var k33 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD, position,
                    oldFrame, k33);

            final var k34 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, speedN, speedE, speedD, position, oldFrame, k34);

            final var k35 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame, k35);

            final var k36 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, velocity, position, oldFrame, k36);

            final var k37 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldFrame, k37);

            final var k38 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldFrame, k38);

            final var k39 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight, k39);

            final var k40 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight, k40);

            final var k41 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitudeAngle, height, oldLatitudeAngle, oldHeight, k41);

            final var k42 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitudeAngle, height, oldLatitudeAngle, oldHeight, k42);

            final var k43 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, heightDistance, oldLatitude, oldHeightDistance, k43);

            final var k44 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, heightDistance, oldLatitude, oldHeightDistance, k44);

            final var k45 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                    k45);

            final var k46 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                    k46);

            assertEquals(k1, k2);
            assertEquals(k1, k3);
            assertEquals(k1, k4);
            assertEquals(k1, k5);
            assertEquals(k1, k6);
            assertEquals(k1, k7);
            assertEquals(k1, k8);
            assertEquals(k1, k9);
            assertEquals(k1, k10);
            assertEquals(k1, k11);
            assertEquals(k1, k12);
            assertEquals(k1, k13);
            assertEquals(k1, k14);
            assertEquals(k1, k15);
            assertEquals(k1, k16);
            assertEquals(k1, k17);
            assertEquals(k1, k18);
            assertEquals(k1, k19);
            assertEquals(k1, k20);
            assertEquals(k1, k21);
            assertEquals(k1, k22);
            assertEquals(k1, k23);
            assertEquals(k1, k24);
            assertEquals(k1, k25);
            assertEquals(k1, k26);
            assertEquals(k1, k27);
            assertEquals(k1, k28);
            assertEquals(k1, k29);
            assertEquals(k1, k30);
            assertEquals(k1, k31);
            assertEquals(k1, k32);
            assertEquals(k1, k33);
            assertEquals(k1, k34);
            assertEquals(k1, k35);
            assertEquals(k1, k36);
            assertEquals(k1, k37);
            assertEquals(k1, k38);
            assertEquals(k1, k39);
            assertEquals(k1, k40);
            assertEquals(k1, k41);
            assertEquals(k1, k42);
            assertEquals(k1, k43);
            assertEquals(k1, k44);
            assertEquals(k1, k45);
            assertEquals(k1, k46);

            final var k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, height, oldLatitude, oldHeight);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testEstimateKinematicsAndReturnNew() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        for (var t = 0; t < TIMES; t++) {
            final var oldFrame = createOldNedFrame();

            final var frame = createNewNedFrame(oldFrame);

            final var c = frame.getCoordinateTransformation();
            final var oldC = oldFrame.getCoordinateTransformation();

            final var vn = frame.getVn();
            final var ve = frame.getVe();
            final var vd = frame.getVd();

            final var oldVn = oldFrame.getVn();
            final var oldVe = oldFrame.getVe();
            final var oldVd = oldFrame.getVd();

            final var latitude = frame.getLatitude();
            final var longitude = frame.getLongitude();
            final var height = frame.getHeight();

            final var oldLatitude = oldFrame.getLatitude();
            final var oldLongitude = oldFrame.getLongitude();
            final var oldHeight = oldFrame.getHeight();

            final var k1 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final var k2 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final var velocity = new NEDVelocity(vn, ve, vd);
            final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
            final var k3 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    velocity, oldVelocity, latitude, height, oldLatitude, oldHeight);

            final var k4 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    velocity, oldVelocity, latitude, height, oldLatitude, oldHeight);

            final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
            final var heightDistance = new Distance(height, DistanceUnit.METER);
            final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
            final var k5 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    velocity, oldVelocity, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

            final var k6 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    velocity, oldVelocity, latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

            final var position = new NEDPosition(latitude, longitude, height);
            final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
            final var k7 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, position, oldPosition);

            final var k8 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, position, oldPosition);

            final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
            final var k9 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

            final var k10 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

            final var k11 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    velocity, oldVelocity, position, oldPosition);

            final var k12 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    velocity, oldVelocity, position, oldPosition);

            final var k13 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldVn, oldVe, oldVd, oldLatitude, oldHeight);

            final var k14 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldVn, oldVe, oldVd, oldLatitude, oldHeight);

            final var k15 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldVelocity, oldLatitude, oldHeight);

            final var k16 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldVelocity, oldLatitude, oldHeight);

            final var k17 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldVelocity, oldLatitude, oldHeight);

            final var k18 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldVelocity, oldLatitude, oldHeight);

            final var k19 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldVn, oldVe, oldVd, oldPosition);

            final var k20 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldVn, oldVe, oldVd, oldPosition);

            final var k21 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldSpeedN, oldSpeedE, oldSpeedD, oldPosition);

            final var k22 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldSpeedN, oldSpeedE, oldSpeedD, oldPosition);

            final var k23 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldC, oldVelocity, oldPosition);

            final var k24 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                    oldVelocity, oldPosition);

            final var k25 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, vn, ve, vd,
                    latitude, height, oldFrame);

            final var k26 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                    latitude, height, oldFrame);

            final var k27 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity,
                    latitude, height, oldFrame);

            final var k28 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                    latitude, height, oldFrame);

            final var k29 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity,
                    latitudeAngle, heightDistance, oldFrame);

            final var k30 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, velocity,
                    latitudeAngle, heightDistance, oldFrame);

            final var k31 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, vn, ve, vd,
                    position, oldFrame);

            final var k32 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, vn, ve, vd,
                    position, oldFrame);

            final var k33 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c,
                    speedN, speedE, speedD, position, oldFrame);

            final var k34 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c,
                    speedN, speedE, speedD, position, oldFrame);

            final var k35 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, velocity,
                    position, oldFrame);

            final var k36 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, velocity, position,
                    oldFrame);

            final var k37 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, frame,
                    oldFrame);

            final var k38 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);

            final var k39 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight);

            final var k40 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, latitude, height, oldLatitude, oldHeight);

            final var k41 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final var k42 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final var k43 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitude, heightDistance, oldLatitude, oldHeightDistance);

            final var k44 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance, oldLatitude, oldHeightDistance);

            final var k45 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                    oldLatitudeAngle, oldHeightDistance);

            final var k46 = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                    oldLatitudeAngle, oldHeightDistance);

            assertEquals(k1, k2);
            assertEquals(k1, k3);
            assertEquals(k1, k4);
            assertEquals(k1, k5);
            assertEquals(k1, k6);
            assertEquals(k1, k7);
            assertEquals(k1, k8);
            assertEquals(k1, k9);
            assertEquals(k1, k10);
            assertEquals(k1, k11);
            assertEquals(k1, k12);
            assertEquals(k1, k13);
            assertEquals(k1, k14);
            assertEquals(k1, k15);
            assertEquals(k1, k16);
            assertEquals(k1, k17);
            assertEquals(k1, k18);
            assertEquals(k1, k19);
            assertEquals(k1, k20);
            assertEquals(k1, k21);
            assertEquals(k1, k22);
            assertEquals(k1, k23);
            assertEquals(k1, k24);
            assertEquals(k1, k25);
            assertEquals(k1, k26);
            assertEquals(k1, k27);
            assertEquals(k1, k28);
            assertEquals(k1, k29);
            assertEquals(k1, k30);
            assertEquals(k1, k31);
            assertEquals(k1, k32);
            assertEquals(k1, k33);
            assertEquals(k1, k34);
            assertEquals(k1, k35);
            assertEquals(k1, k36);
            assertEquals(k1, k37);
            assertEquals(k1, k38);
            assertEquals(k1, k39);
            assertEquals(k1, k40);
            assertEquals(k1, k41);
            assertEquals(k1, k42);
            assertEquals(k1, k43);
            assertEquals(k1, k44);
            assertEquals(k1, k45);
            assertEquals(k1, k46);

            final var k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    latitude, height, oldLatitude, oldHeight);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldHeight = oldFrame.getHeight();

        assertThrows(IllegalArgumentException.class, () -> NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                -1.0, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight));
    }

    @Test
    void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldHeight = oldFrame.getHeight();

        c.setDestinationType(FrameType.BODY_FRAME);

        assertThrows(IllegalArgumentException.class, () -> NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude,
                oldHeight));
    }

    @Test
    void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldHeight = oldFrame.getHeight();

        oldC.setDestinationType(FrameType.BODY_FRAME);

        assertThrows(IllegalArgumentException.class, () -> NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude,
                oldHeight));
    }

    @Test
    void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues()
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final var oldFrame = createOldNedFrame();

        final var frame = createNewNedFrame(oldFrame);

        final var c = frame.getCoordinateTransformation();
        final var oldC = oldFrame.getCoordinateTransformation();

        final var vn = frame.getVn();
        final var ve = frame.getVe();
        final var vd = frame.getVd();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var latitude = frame.getLatitude();
        final var height = frame.getHeight();

        final var oldLatitude = oldFrame.getLatitude();
        final var oldHeight = oldFrame.getHeight();

        final var k = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(0.0, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);

        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        final var k2 = estimateKinematics(0.0, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height,
                oldLatitude, oldHeight);

        assertTrue(k2.equals(k, 0.0));
    }

    @Test
    void testCompareKinematics() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException {
        for (var t = 0; t < TIMES; t++) {
            final var oldNedFrame = createOldNedFrame();
            final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                    oldEcefFrame);

            final var newNedFrame = createNewNedFrame(oldNedFrame);
            final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newNedFrame);
            final var newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                    newEcefFrame);

            final var nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newNedFrame,
                    oldNedFrame);
            final var ecefK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                    newEcefFrame, oldEcefFrame);
            final var eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEciFrame,
                    oldEciFrame);

            final var nedSpecificForceNorm = nedK.getSpecificForceNorm();
            final var ecefSpecificForceNorm = ecefK.getSpecificForceNorm();
            final var eciSpecificForceNorm = eciK.getSpecificForceNorm();

            final var nedAngularRateNorm = nedK.getAngularRateNorm();
            final var ecefAngularRateNorm = ecefK.getAngularRateNorm();
            final var eciAngularRateNorm = eciK.getAngularRateNorm();

            assertEquals(ecefSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);
            assertEquals(eciSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);

            assertEquals(ecefAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);
            assertEquals(eciAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);

            assertEquals(ecefK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateX(), nedK.getAngularRateX(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateY(), nedK.getAngularRateY(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateZ(), nedK.getAngularRateZ(), LARGE_ABSOLUTE_ERROR);

            assertEquals(eciK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateX(), nedK.getAngularRateX(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateY(), nedK.getAngularRateY(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateZ(), nedK.getAngularRateZ(), LARGE_ABSOLUTE_ERROR);

            assertTrue(ecefK.equals(nedK, LARGE_ABSOLUTE_ERROR));
            assertTrue(eciK.equals(nedK, LARGE_ABSOLUTE_ERROR));
        }
    }

    private static NEDFrame createOldNedFrame() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final var randomizer = new UniformRandomizer();

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        return new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
    }

    private static NEDFrame createNewNedFrame(final NEDFrame oldFrame) throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final var oldLatitude = oldFrame.getLatitude();
        final var oldLongitude = oldFrame.getLongitude();
        final var oldHeight = oldFrame.getHeight();

        final var oldVn = oldFrame.getVn();
        final var oldVe = oldFrame.getVe();
        final var oldVd = oldFrame.getVd();

        final var oldC = oldFrame.getCoordinateTransformation();

        final var oldRoll = oldC.getRollEulerAngle();
        final var oldPitch = oldC.getPitchEulerAngle();
        final var oldYaw = oldC.getYawEulerAngle();

        final var randomizer = new UniformRandomizer();

        final var latitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var longitudeVariation = Math.toRadians(randomizer.nextDouble(MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final var heightVariation = randomizer.nextDouble(MIN_HEIGHT_VARIATION, MAX_HEIGHT_VARIATION);

        final var vnVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var veVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);
        final var vdVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION, MAX_VELOCITY_VARIATION);

        final var rollVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES, 
                MAX_ANGLE_VARIATION_DEGREES));
        final var pitchVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));
        final var yawVariation = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                MAX_ANGLE_VARIATION_DEGREES));

        final var latitude = oldLatitude + latitudeVariation;
        final var longitude = oldLongitude + longitudeVariation;
        final var height = oldHeight + heightVariation;

        final var vn = oldVn + vnVariation;
        final var ve = oldVe + veVariation;
        final var vd = oldVd + vdVariation;

        final var roll = oldRoll + rollVariation;
        final var pitch = oldPitch + pitchVariation;
        final var yaw = oldYaw + yawVariation;

        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }

    private static BodyKinematics estimateKinematics(
            final double timeInterval, final CoordinateTransformation c, final CoordinateTransformation oldC,
            final double vn, final double ve, final double vd,
            final double oldVn, final double oldVe, final double oldVd, final double latitude, final double height,
            final double oldLatitude, final double oldHeight) throws WrongSizeException, RankDeficientMatrixException,
            DecomposerException {

        if (timeInterval > 0.0) {
            final var oldVebn = new Matrix(3, 1);
            oldVebn.setElementAtIndex(0, oldVn);
            oldVebn.setElementAtIndex(1, oldVe);
            oldVebn.setElementAtIndex(2, oldVd);

            final var vEbn = new Matrix(3, 1);
            vEbn.setElementAtIndex(0, vn);
            vEbn.setElementAtIndex(1, ve);
            vEbn.setElementAtIndex(2, vd);

            final var cBn = c.getMatrix();
            final var oldCbn = oldC.getMatrix();

            final var omegaIe = Constants.EARTH_ROTATION_RATE;

            // From (2.123), determine the angular rate of the ECEF frame with respect
            // to the ECI frame, resolved about NED
            Matrix tmp = new Matrix(3, 1);
            tmp.setElementAtIndex(0, Math.cos(oldLatitude));
            tmp.setElementAtIndex(1, 0.0);
            tmp.setElementAtIndex(2, -Math.sin(oldLatitude));

            final var omegaIen = tmp.multiplyByScalarAndReturnNew(omegaIe);

            // From (5.44), determine the angular rate of the NED frame with respect
            // to the ECEF frame, resolved about NED
            final var oldRadiiOfCurvature = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
            final var oldRn = oldRadiiOfCurvature.getRn();
            final var oldRe = oldRadiiOfCurvature.getRe();

            final var radiiOfCurvature = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(latitude);
            final var rn = radiiOfCurvature.getRn();
            final var re = radiiOfCurvature.getRe();

            final var oldOmegaEnN = new Matrix(3, 1);
            oldOmegaEnN.setElementAtIndex(0, oldVebn.getElementAtIndex(1) / (oldRe + oldHeight));
            oldOmegaEnN.setElementAtIndex(1, -oldVebn.getElementAtIndex(0) / (oldRn + oldHeight));
            oldOmegaEnN.setElementAtIndex(2,
                    -oldVebn.getElementAtIndex(1) * Math.tan(oldLatitude) / (oldRe + oldHeight));

            final var omegaEnN = new Matrix(3, 1);
            omegaEnN.setElementAtIndex(0, vEbn.getElementAtIndex(1) / (re + height));
            omegaEnN.setElementAtIndex(1, -vEbn.getElementAtIndex(0) / (rn + height));
            omegaEnN.setElementAtIndex(2, -vEbn.getElementAtIndex(1) * Math.tan(latitude) / (re + height));

            // Get coordinate transformation matrix from the old attitude with respect
            // to an inertial frame to the new using (5.77)

            //cOldNew = cBn' * (I - timeInterval*skew(omegaIen + 0.5*omegaEnN + 0.5*oldOmegaEnN)) * oldCbn
            final var cOldNew = cBn.transposeAndReturnNew().multiplyAndReturnNew(
                    Matrix.identity(3, 3).subtractAndReturnNew(
                            Utils.skewMatrix(omegaIen.addAndReturnNew(
                                    omegaEnN.multiplyByScalarAndReturnNew(0.5)).addAndReturnNew(
                                    oldOmegaEnN.multiplyByScalarAndReturnNew(0.5)))
                                    .multiplyByScalarAndReturnNew(timeInterval))
                            .multiplyAndReturnNew(oldCbn));

            // Calculate the approximate angular rate with respect to an inertial frame
            final var alphaIbb = new Matrix(3, 1);
            alphaIbb.setElementAtIndex(0, 0.5 * (
                    cOldNew.getElementAt(1, 2) - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1, 0.5 * (
                    cOldNew.getElementAt(2, 0) - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2, 0.5 * (
                    cOldNew.getElementAt(0, 1) - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final var temp = Math.acos(0.5 * (cOldNew.getElementAt(0, 0) 
                    + cOldNew.getElementAt(1, 1) 
                    + cOldNew.getElementAt(2, 2) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final var omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(1.0 / timeInterval);

            // Calculate the specific force resolved about ECEF-frame axes
            // From (5.54),
            final var gravity = NEDGravityEstimator.estimateGravityAndReturnNew(oldLatitude, oldHeight);
            final var g = gravity.asMatrix();
            final var fIbn = vEbn.subtractAndReturnNew(oldVebn);
            fIbn.multiplyByScalar(1.0 / timeInterval);
            fIbn.subtract(g);
            fIbn.add(Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen.multiplyByScalarAndReturnNew(2.0)))
                    .multiplyAndReturnNew(oldVebn));

            // Calculate the average body-to-NED coordinate transformation
            // matrix over the update interval using (5.84) and (5.86)
            final var magAlpha = Utils.normF(alphaIbb);
            final var skewAlphaIbb = Utils.skewMatrix(alphaIbb);
            final Matrix aveCbn;
            if (magAlpha > ALPHA_THRESHOLD) {
                // aveCbn = oldCbn * (I + (1 - cos(magAlpha))/magAlpha^2*AlphaIbb +
                //   (1 - sin(magAlpha)/magAlpha)/magAlpha^2*AlphaIbb*AlphaIbb)
                //   -0.5 * skew(oldOmegaEnn + omegaIen)*oldCbn
                aveCbn = oldCbn.multiplyAndReturnNew(
                        Matrix.identity(3, 3).addAndReturnNew(
                                skewAlphaIbb.multiplyByScalarAndReturnNew(
                                        (1.0 - Math.cos(magAlpha)) / Math.pow(magAlpha, 2.0))
                        ).addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(
                                (1.0 - Math.sin(magAlpha) / magAlpha) / Math.pow(magAlpha, 2.0))
                                .multiplyAndReturnNew(skewAlphaIbb))).subtractAndReturnNew(
                        Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen))
                                .multiplyByScalarAndReturnNew(0.5)
                                .multiplyAndReturnNew(oldCbn));
            } else {
                aveCbn = oldCbn.subtractAndReturnNew(Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen))
                        .multiplyByScalarAndReturnNew(0.5).multiplyAndReturnNew(oldCbn));
            }

            // Transform specific force to body-frame resolving axes using (5.81)
            final var fIbb = Utils.inverse(aveCbn).multiplyAndReturnNew(fIbn);

            final var fx = fIbb.getElementAtIndex(0);
            final var fy = fIbb.getElementAtIndex(1);
            final var fz = fIbb.getElementAtIndex(2);

            final var angularRateX = omegaIbb.getElementAtIndex(0);
            final var angularRateY = omegaIbb.getElementAtIndex(1);
            final var angularRateZ = omegaIbb.getElementAtIndex(2);

            return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        } else {
            // If the time interval is zero, set the angular rate and specific force to zero
            return new BodyKinematics();
        }
    }
}
