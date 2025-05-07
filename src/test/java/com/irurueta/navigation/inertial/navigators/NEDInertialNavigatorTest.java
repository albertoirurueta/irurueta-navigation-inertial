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
package com.irurueta.navigation.inertial.navigators;

import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator;
import com.irurueta.navigation.inertial.estimators.NEDKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class NEDInertialNavigatorTest {
    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;
    
    private static final double MIN_HEIGHT = -10.0;
    private static final double MAX_HEIGHT = 10.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_SPECIFIC_FORCE = -12.0;
    private static final double MAX_SPECIFIC_FORCE = 12.0;

    private static final double MIN_ANGULAR_RATE_DEGREES_PER_SECOND = -5.0;
    private static final double MAX_ANGULAR_RATE_DEGREES_PER_SECOND = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 100;

    private static final double ACCURACY_THRESHOLD = 1e-6;

    @Test
    void testNavigateNEDWhenInvalidCoordinateTransformationMatrix() {
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var result = new NEDFrame();
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> NEDInertialNavigator.navigateNED(
                0.0, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result));
    }

    @Test
    void testNavigate() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {

        final var navigator = new NEDInertialNavigator();

        final var randomizer = new UniformRandomizer();

        final var oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final var oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var oldC = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT, oldVn, oldVe, oldVd, oldC);

        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);

        final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result1 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final var result3 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result3);

        final var result4 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result4);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result5 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final var result6 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final var result7 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result7);

        final var result8 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result8);

        final var result9 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                kinematics, result9);

        final var result10 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, kinematics,
                result10);

        final var result11 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, kinematics, result11);

        final var result12 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, kinematics, result12);

        final var result13 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, kinematics,
                result13);

        final var result14 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, kinematics, result14);

        final var result15 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, kinematics, result15);

        final var result16 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity, kinematics, result16);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final var result17 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result17);

        final var result18 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result18);

        final var result19 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result19);

        final var result20 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result20);

        final var result21 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldVn, oldVe, oldVd, kinematics, result21);

        final var result22 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldVn, oldVe, oldVd, kinematics, result22);

        final var result23 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, kinematics, result23);

        final var result24 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity,
                kinematics, result24);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var result25 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result25);

        final var result26 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result26);

        final var result27 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result27);

        final var result28 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final var result29 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result29);

        final var result30 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result30);

        final var result31 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result31);

        final var result32 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result32);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result33 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result33);

        final var result34 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result34);

        final var result35 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result35);

        final var result36 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result36);

        final var result37 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result37);

        final var result38 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result38);

        final var result39 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result39);

        final var result40 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result40);

        final var result41 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, 
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result41);

        final var result42 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result42);

        final var result43 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result43);

        final var result44 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result44);

        final var result45 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result45);

        final var result46 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result46);

        final var result47 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result47);

        final var result48 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result48);

        final var result49 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result49);

        final var result50 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result50);

        final var result51 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final var result52 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final var result53 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, 
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result53);

        final var result54 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result54);

        final var result55 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final var result56 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final var result57 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                result57);

        final var result58 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result58);

        final var result59 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result59);

        final var result60 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result60);

        final var result61 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final var result62 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final var result63 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final var result64 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result64);

        final var result65 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final var result66 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final var result67 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result67);

        final var result68 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result68);

        final var result69 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result69);

        final var result70 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result70);

        final var result71 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result71);

        final var result72 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, kinematics, result72);

        final var result73 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result73);

        final var result74 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, result74);

        final var result75 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result75);

        final var result76 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result76);

        final var result77 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result77);

        final var result78 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result78);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
    }

    @Test
    void testNavigateAndReturnNew() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        final var navigator = new NEDInertialNavigator();

        final var randomizer = new UniformRandomizer();

        final var oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final var oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var oldC = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT, oldVn, oldVe, oldVd, oldC);

        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);

        final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result1 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, 
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final var result3 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result4 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result5 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, 
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result6 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result7 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result8 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result9 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, 
                oldC, oldVn, oldVe, oldVd, kinematics);

        final var result10 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final var result11 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVn, oldVe, oldVd, kinematics);

        final var result12 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                kinematics);

        final var result13 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, kinematics);

        final var result14 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVelocity, kinematics);

        final var result15 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                kinematics);

        final var result16 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity, kinematics);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final var result17 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result18 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result19 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result20 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result21 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result22 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result23 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, kinematics);

        final var result24 = navigator.navigateAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, kinematics);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var result25 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result26 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result27 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result28 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result29 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, 
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result30 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result31 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result32 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result33 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, 
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result34 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result35 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result36 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result37 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result38 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result39 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result40 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result41 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result42 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result43 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe,
                oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result44 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result45 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result46 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result47 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result48 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result49 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result50 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result51 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result52 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result53 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result54 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, 
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result55 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result56 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result57 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result58 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result59 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result60 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result61 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result62 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result63 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result64 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result65 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result66 = navigator.navigateAndReturnNew(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result67 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final var result68 = navigator.navigateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result69 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result70 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result71 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final var result72 = navigator.navigateAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result73 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result74 = navigator.navigateAndReturnNew(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result75 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result76 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result77 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result78 = navigator.navigateAndReturnNew(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
    }

    @Test
    void testNavigateNED() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {

        final var randomizer = new UniformRandomizer();

        final var oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final var oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var oldC = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT, oldVn, oldVe, oldVd, oldC);

        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);

        final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result1 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD,
                result2);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final var result3 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result3);

        final var result4 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result4);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result5 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD,
                result5);

        final var result6 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result6);

        final var result7 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result7);

        final var result8 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result8);

        final var result9 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldVn, oldVe, oldVd, kinematics, result9);

        final var result10 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                kinematics, result10);

        final var result11 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, kinematics,
                result11);

        final var result12 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, kinematics, result12);

        final var result13 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                kinematics, result13);

        final var result14 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, kinematics,
                result14);

        final var result15 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, kinematics, result15);

        final var result16 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity, kinematics, result16);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final var result17 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result17);

        final var result18 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result18);

        final var result19 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result19);

        final var result20 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result20);

        final var result21 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, kinematics, result21);

        final var result22 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVn, oldVe, oldVd, kinematics, result22);

        final var result23 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVelocity, kinematics, result23);

        final var result24 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, kinematics, result24);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var result25 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result25);

        final var result26 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result26);

        final var result27 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD,
                result27);

        final var result28 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result28);

        final var result29 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result29);

        final var result30 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result30);

        final var result31 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result31);

        final var result32 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result32);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result33 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result33);

        final var result34 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result34);

        final var result35 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result35);

        final var result36 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result36);

        final var result37 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result37);

        final var result38 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result38);

        final var result39 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result39);

        final var result40 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result40);

        final var result41 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result41);

        final var result42 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result42);

        final var result43 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result43);

        final var result44 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result44);

        final var result45 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result45);

        final var result46 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result46);

        final var result47 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result47);

        final var result48 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result48);

        final var result49 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result49);

        final var result50 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result50);

        final var result51 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result51);

        final var result52 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result52);

        final var result53 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result53);

        final var result54 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result54);

        final var result55 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result55);

        final var result56 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result56);

        final var result57 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result57);

        final var result58 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result58);

        final var result59 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result59);

        final var result60 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result60);

        final var result61 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result61);

        final var result62 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result62);

        final var result63 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result63);

        final var result64 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result64);

        final var result65 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result65);

        final var result66 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD, result66);

        final var result67 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result67);

        final var result68 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result68);

        final var result69 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result69);

        final var result70 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result70);

        final var result71 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result71);

        final var result72 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, kinematics, result72);

        final var result73 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result73);

        final var result74 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result74);

        final var result75 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result75);

        final var result76 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result76);

        final var result77 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result77);

        final var result78 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result78);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
    }

    @Test
    void testNavigateNEDAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        final var randomizer = new UniformRandomizer();

        final var oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final var oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var oldC = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT, oldVn, oldVe, oldVd, oldC);

        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond, AngularSpeedUnit.DEGREES_PER_SECOND);

        final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result1 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final var result3 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result4 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result5 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result6 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result7 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result8 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity, 
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result9 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result10 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result11 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final var result12 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVn, oldVe, oldVd, kinematics);

        final var result13 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, kinematics);

        final var result14 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVelocity, kinematics);

        final var result15 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, kinematics);

        final var result16 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldVelocity, kinematics);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final var result17 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result18 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result19 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result20 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result21 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result22 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final var result23 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, kinematics);

        final var result24 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, kinematics);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var result25 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result26 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result27 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result28 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result29 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result30 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result31 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var result32 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result33 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result34 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result35 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result36 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result37 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result38 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVelocity, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result39 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result40 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result41 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result42 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, 
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result43 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result44 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVn, oldVe, oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result45 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVelocity, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result46 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, 
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result47 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result48 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result49 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result50 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result51 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result52 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result53 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result54 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result55 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result56 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result57 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result58 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result59 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result60 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result61 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result62 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result63 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result64 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result65 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitude, oldLongitude, HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result66 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldLatitude, oldLongitude, 
                HEIGHT, oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, NEDInertialNavigator.DEFAULT_ACCURACY_THRESHOLD);

        final var result67 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD, 
                kinematics);

        final var result68 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, 
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD, 
                kinematics);

        final var result69 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result70 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result71 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final var result72 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result73 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result74 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result75 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result76 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result77 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result78 = NEDInertialNavigator.navigateNEDAndReturnNew(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

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
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
    }

    @Test
    void testNavigateFreeFallingBody() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = 0.0;
            final var ve = 0.0;
            final var vd = 0.0;

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var gravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
            final var fx = gravity.getGn();
            final var fy = gravity.getGe();
            final var fz = gravity.getGd();

            final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, 
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                    angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                    angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                    angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

            final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                    kinematics);

            // Because the body is in free fall, its latitude and longitude does not change
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(), ABSOLUTE_ERROR);

            // Since the body is falling, the new height is smaller than the initial one
            assertTrue(newFrame.getHeight() < oldFrame.getHeight());

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateZeroTimeInterval() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var gravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
            final var fx = gravity.getGn();
            final var fy = gravity.getGe();
            final var fz = gravity.getGd();

            final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, 
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                    angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                    angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                    angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

            final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(0.0, oldFrame, kinematics);

            // Because the time interval is zero, the body hasn't moved
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(oldFrame.getHeight(), newFrame.getHeight(), ABSOLUTE_ERROR);
            assertTrue(oldFrame.equals(newFrame, LARGE_ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWithInitialVelocityAndNoSpecificForce() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var fx = 0.0;
            final var fy = 0.0;
            final var fz = 0.0;

            final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond, 
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                    angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                    angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                    angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

            final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                    kinematics);

            // Because no specific force is applied to the body, it will keep its inertia
            // (initial speed). This is approximate, since as the body moves, the amount
            // of gravity applied to it changes as well
            final var oldSpeed = oldFrame.getVelocityNorm();
            final var newSpeed = newFrame.getVelocityNorm();

            final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldFrame);
            final var newEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(newFrame);

            final var oldPosition = oldEcefFrame.getPosition();
            final var newPosition = newEcefFrame.getPosition();

            final var distance = oldPosition.distanceTo(newPosition);

            final var estimatedSpeed = distance / TIME_INTERVAL_SECONDS;

            assertEquals(estimatedSpeed, newSpeed, VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(oldSpeed, newSpeed, 2.0 * VERY_LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWithNoInitialVelocityAndNoSpecificForce() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = 0.0;
            final var ve = 0.0;
            final var vd = 0.0;

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

            final var fx = 0.0;
            final var fy = 0.0;
            final var fz = 0.0;

            final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                    angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                    angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                    angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

            final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            // Because no forces are applied to the body and the body has no initial velocity,
            // the body will remain in the same position
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(oldFrame.getHeight(), newFrame.getHeight(), VERY_LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testCompareNavigations() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
            final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldFrame);

            final var gravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);
            final var fx = gravity.getGn();
            final var fy = gravity.getGe();
            final var fz = gravity.getGd();

            final var angularRateXDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateYDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final var angularRateZDegreesPerSecond = randomizer.nextDouble(MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final var angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final var angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final var angularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(),
                    angularSpeedX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(),
                    angularSpeedY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
            final var angularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(),
                    angularSpeedZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

            final var kinematics = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

            final var newFrame1 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            final var newEcefFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame,
                    kinematics);
            final var newFrame2 = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newEcefFrame);

            // compare
            assertTrue(newFrame1.equals(newFrame2, LARGE_ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWhenFrameRemainsConstant() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var vn = 0.0;
            final var ve = 0.0;
            final var vd = 0.0;

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var q = new Quaternion(roll, pitch, yaw);

            final var m = q.asInhomogeneousMatrix();
            final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var bodyKinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    oldFrame, oldFrame);

            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    bodyKinematics);

            assertTrue(oldFrame.equals(newFrame, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWhenFrameRemainsConstant2() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var vn = 0.0;
        final var ve = 0.0;
        final var vd = 0.0;

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var bodyKinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                oldFrame, oldFrame);
        final var initialFrame = new NEDFrame(oldFrame);

        for (var t = 0; t < TIMES; t++) {
            final var newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    bodyKinematics, ACCURACY_THRESHOLD);

            assertTrue(initialFrame.equals(newFrame, ABSOLUTE_ERROR));

            newFrame.copyTo(oldFrame);
        }
    }
}
