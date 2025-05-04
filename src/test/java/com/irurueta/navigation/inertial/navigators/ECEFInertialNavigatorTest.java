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

import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ECEFInertialNavigatorTest {
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

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final double NAVIGATE_ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 100;

    @Test
    void testNavigateEcefWhenInvalidCoordinateTransformationMatrix() {
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var result = new ECEFFrame();
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> ECEFInertialNavigator.navigateECEF(
                0.0, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, result));
    }

    @Test
    void testNavigate() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
        final var navigator = new ECEFInertialNavigator();

        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var oldX = oldFrame.getX();
        final var oldY = oldFrame.getY();
        final var oldZ = oldFrame.getZ();
        final var oldC = oldFrame.getCoordinateTransformation();
        final var oldVx = oldFrame.getVx();
        final var oldVy = oldFrame.getVy();
        final var oldVz = oldFrame.getVz();

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

        final var result1 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final var oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final var result3 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result3);

        final var result4 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result4);

        final var oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var result5 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final var result6 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final var result7 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result7);

        final var result8 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result8);

        final var result9 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics, result9);

        final var result10 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics, result10);

        final var result11 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz, kinematics, result11);

        final var result12 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, kinematics, result12);

        final var result13 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, kinematics, result13);

        final var result14 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, kinematics, result14);

        final var result15 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, kinematics, result15);

        final var result16 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, kinematics, result16);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result17 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final var result18 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final var result19 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result19);

        final var result20 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result20);

        final var result21 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result21);

        final var result22 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result22);

        final var result23 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity, kinematics, result23);

        final var result24 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldEcefVelocity, kinematics, result24);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result25 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result25);

        final var result26 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result26);

        final var result27 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result27);

        final var result28 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final var result29 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result29);

        final var result30 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, 
                kinematics, result30);

        final var result31 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                kinematics, result31);

        final var result32 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, kinematics,
                result32);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result33 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result33);

        final var result34 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result34);

        final var result35 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result35);

        final var result36 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result36);

        final var result37 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result37);

        final var result38 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result38);

        final var result39 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result39);

        final var result40 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result40);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result41 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result41);

        final var result42 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result42);

        final var result43 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result43);

        final var result44 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result44);

        final var result45 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result45);

        final var result46 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result46);

        final var result47 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result47);

        final var result48 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result48);

        final var result49 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result49);

        final var result50 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result50);

        final var result51 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final var result52 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final var result53 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result53);

        final var result54 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result54);

        final var result55 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final var result56 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final var result57 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result57);

        final var result58 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result58);

        final var result59 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result59);

        final var result60 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result60);

        final var result61 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final var result62 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final var result63 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final var result64 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result64);

        final var result65 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final var result66 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final var result67 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result67);

        final var result68 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result68);

        final var result69 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result69);

        final var result70 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result70);

        final var result71 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result71);

        final var result72 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result72);

        final var result73 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result73);

        final var result74 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result74);

        final var result75 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result75);

        final var result76 = new ECEFFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result76);

        final var result77 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result77);

        final var result78 = new ECEFFrame();
        navigator.navigate(timeInterval, oldFrame, kinematics, result78);

        final var result79 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result79);

        final var result80 = new ECEFFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, result80);

        final var result81 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result81);

        final var result82 = new ECEFFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result82);

        final var result83 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result83);

        final var result84 = new ECEFFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result84);

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
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    void testNavigateAndReturnNew() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final var navigator = new ECEFInertialNavigator();

        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var oldX = oldFrame.getX();
        final var oldY = oldFrame.getY();
        final var oldZ = oldFrame.getZ();
        final var oldC = oldFrame.getCoordinateTransformation();
        final var oldVx = oldFrame.getVx();
        final var oldVy = oldFrame.getVy();
        final var oldVz = oldFrame.getVz();

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

        final var result1 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final var result3 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result4 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var result5 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result6 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result7 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result8 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result9 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result10 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final var result11 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result12 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final var result13 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldEcefVelocity, kinematics);

        final var result14 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                kinematics);

        final var result15 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldEcefVelocity, kinematics);

        final var result16 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                kinematics);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result17 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result18 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result19 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result20 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldEcefVelocity, 
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result21 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result22 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final var result23 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity,
                kinematics);

        final var result24 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldEcefVelocity,
                kinematics);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result25 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result26 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result27 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result28 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result29 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var result30 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldVx, oldVy, oldVz, kinematics);

        final var result31 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, kinematics);

        final var result32 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldEcefVelocity, kinematics);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result33 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result34 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result35 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result36 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result37 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result38 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result39 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result40 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result41 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result42 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result43 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result44 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result45 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result46 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result47 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result48 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result49 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result50 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result51 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result52 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result53 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result54 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result55 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result56 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result57 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result58 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result59 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result60 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result61 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result62 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result63 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result64 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result65 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result66 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result67 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result68 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result69 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result70 = navigator.navigateAndReturnNew(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result71 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result72 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result73 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result74 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result75 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result76 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result77 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final var result78 = navigator.navigateAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result79 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result80 = navigator.navigateAndReturnNew(timeInterval, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result81 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result82 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result83 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result84 = navigator.navigateAndReturnNew(timeInterval, oldFrame,
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
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    void testNavigateEcef() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var oldX = oldFrame.getX();
        final var oldY = oldFrame.getY();
        final var oldZ = oldFrame.getZ();
        final var oldC = oldFrame.getCoordinateTransformation();
        final var oldVx = oldFrame.getVx();
        final var oldVy = oldFrame.getVy();
        final var oldVz = oldFrame.getVz();

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

        final var result1 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final var oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final var result3 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result3);

        final var result4 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result4);

        final var oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var result5 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final var result6 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final var result7 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result7);

        final var result8 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result8);

        final var result9 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result9);

        final var result10 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result10);

        final var result11 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz, 
                kinematics, result11);

        final var result12 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result12);

        final var result13 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, kinematics,
                result13);

        final var result14 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, kinematics, result14);

        final var result15 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, kinematics,
                result15);

        final var result16 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, kinematics, result16);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result17 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final var result18 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final var result19 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result19);

        final var result20 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result20);

        final var result21 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result21);

        final var result22 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result22);

        final var result23 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity, kinematics,
                result23);

        final var result24 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition, oldC, oldEcefVelocity, kinematics, result24);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result25 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result25);

        final var result26 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result26);

        final var result27 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result27);

        final var result28 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result28);

        final var result29 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result29);

        final var result30 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result30);

        final var result31 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result31);

        final var result32 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result32);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result33 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result33);

        final var result34 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result34);

        final var result35 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result35);

        final var result36 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result36);

        final var result37 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result37);

        final var result38 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result38);

        final var result39 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result39);

        final var result40 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result40);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result41 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result41);

        final var result42 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result42);

        final var result43 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result43);

        final var result44 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result44);

        final var result45 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result45);

        final var result46 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result46);

        final var result47 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result47);

        final var result48 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result48);

        final var result49 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result49);

        final var result50 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result50);

        final var result51 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final var result52 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final var result53 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result53);

        final var result54 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result54);

        final var result55 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final var result56 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final var result57 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result57);

        final var result58 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result58);

        final var result59 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result59);

        final var result60 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result60);

        final var result61 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final var result62 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final var result63 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final var result64 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result64);

        final var result65 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final var result66 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final var result67 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result67);

        final var result68 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result68);

        final var result69 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result69);

        final var result70 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result70);

        final var result71 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result71);

        final var result72 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result72);

        final var result73 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result73);

        final var result74 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result74);

        final var result75 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result75);

        final var result76 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result76);

        final var result77 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result77);

        final var result78 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldFrame, kinematics, result78);

        final var result79 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result79);

        final var result80 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result80);

        final var result81 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result81);

        final var result82 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result82);

        final var result83 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result83);

        final var result84 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result84);

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
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    void testNavigateEcefAndReturnNew() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var q = new Quaternion(roll, pitch, yaw);

        final var m = q.asInhomogeneousMatrix();
        final var c = new CoordinateTransformation(m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

        final var oldX = oldFrame.getX();
        final var oldY = oldFrame.getY();
        final var oldZ = oldFrame.getZ();
        final var oldC = oldFrame.getCoordinateTransformation();
        final var oldVx = oldFrame.getVx();
        final var oldVy = oldFrame.getVy();
        final var oldVz = oldFrame.getVz();


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

        final var result1 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final var result3 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result4 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final var result5 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result6 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result7 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result8 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result9 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldVx, oldVy, oldVz, kinematics);

        final var result10 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result11 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldVx, oldVy, oldVz, kinematics);

        final var result12 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final var result13 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldEcefVelocity, kinematics);

        final var result14 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, kinematics);

        final var result15 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldEcefVelocity, kinematics);

        final var result16 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, kinematics);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result17 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result18 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result19 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result20 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldPosition, oldC,
                oldEcefVelocity, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result21 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result22 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result23 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldEcefVelocity, kinematics);

        final var result24 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldPosition, oldC, 
                oldEcefVelocity, kinematics);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result25 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result26 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result27 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result28 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result29 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var result30 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var result31 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, kinematics);

        final var result32 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, kinematics);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result33 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result34 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result35 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result36 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result37 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result38 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result39 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result40 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result41 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result42 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result43 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result44 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result45 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result46 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result47 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result48 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result49 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result50 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result51 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result52 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result53 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result54 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result55 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition, 
                oldC, oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result56 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result57 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result58 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result59 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result60 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result61 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result62 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result63 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result64 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity, 
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result65 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result66 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result67 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result68 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result69 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result70 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result71 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, 
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result72 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result73 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result74 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result75 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result76 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result77 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, 
                kinematics);

        final var result78 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result79 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result80 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result81 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result82 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result83 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result84 = ECEFInertialNavigator.navigateECEFAndReturnNew(timeInterval, oldFrame,
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
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    void testNavigateFreeFallingBody() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException, WrongSizeException {

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

            final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldFrame);
            final var g = gravity.asMatrix();
            final var cbe = oldFrame.getCoordinateTransformationMatrix();
            final var invCbe = cbe.transposeAndReturnNew();
            final var f = invCbe.multiplyAndReturnNew(g);

            final var fx = f.getElementAtIndex(0);
            final var fy = f.getElementAtIndex(1);
            final var fz = f.getElementAtIndex(2);

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

            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            final var newNedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newFrame);

            // Because the body is in free fall, its latitude and longitude does not change
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(), ABSOLUTE_ERROR);

            // Since the body is falling, the new height is smaller than the initial one
            assertTrue(newNedFrame.getHeight() < oldNedFrame.getHeight());

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateZeroTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException, WrongSizeException {

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

            final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldFrame);
            final var g = gravity.asMatrix();
            final var cbe = oldFrame.getCoordinateTransformation().getMatrix();
            final var f = cbe.multiplyAndReturnNew(g);

            final var fx = f.getElementAtIndex(0);
            final var fy = f.getElementAtIndex(1);
            final var fz = f.getElementAtIndex(2);

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

            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(0.0, oldFrame, kinematics);

            final var newNedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newFrame);

            // Because the time interval is zero, the body hasn't moved
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getHeight(), newNedFrame.getHeight(), ABSOLUTE_ERROR);
            assertTrue(oldNedFrame.equals(newNedFrame, ABSOLUTE_ERROR));

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

            final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            final var newNedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newFrame);

            // Because no specific force is applied to body, it will keep its inertia
            // (initial speed). This is approximate, since as the body moves the amount
            // of gravity applied to it changes as well
            final var oldSpeed = oldNedFrame.getVelocityNorm();
            final var newSpeed = newNedFrame.getVelocityNorm();

            final var oldPosition = oldFrame.getPosition();
            final var newPosition = newFrame.getPosition();
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

            final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            final var newNedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newFrame);

            // Because no forces are applied to the body and the body has no initial velocity,
            // the body will remain on the same position
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(), ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(), ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getHeight(), newNedFrame.getHeight(), LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWhenFrameRemainsConstant() throws InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
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

            final var c = new CoordinateTransformation(q, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var bodyKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    oldFrame, oldFrame);

            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    bodyKinematics);

            assertTrue(oldFrame.equals(newFrame, NAVIGATE_ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }

    @Test
    void testNavigateWhenFrameRemainsConstant2() throws InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
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

        final var c = new CoordinateTransformation(q, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final var oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
        final var oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var bodyKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                oldFrame, oldFrame);
        final var initialFrame = new ECEFFrame(oldFrame);

        for (var t = 0; t < TIMES; t++) {
            final var newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    bodyKinematics);

            assertTrue(initialFrame.equals(newFrame, NAVIGATE_ABSOLUTE_ERROR));

            newFrame.copyTo(oldFrame);
        }
    }
}
