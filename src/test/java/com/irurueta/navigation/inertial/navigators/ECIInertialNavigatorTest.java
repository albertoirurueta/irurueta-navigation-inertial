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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.ECItoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ECIInertialNavigatorTest {
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

    private static final int TIMES = 100;

    @Test
    void testNavigateECiWhenInvalidCoordinateTransformationMatrix() {
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var result = new ECIFrame();
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> ECIInertialNavigator.navigateECI(
                0.0, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, result));
    }

    @Test
    void testNavigate() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
        final var navigator = new ECIInertialNavigator();

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
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

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

        final var result1 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final var result3 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics, result3);

        final var result4 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics, result4);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result5 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final var result6 = new ECIFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final var result7 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result7);

        final var result8 = new ECIFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result9 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result9);

        final var result10 = new ECIFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, 
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result10);

        final var result11 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result11);

        final var result12 = new ECIFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, 
                kinematics, result12);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result13 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result13);

        final var result14 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result14);

        final var result15 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result15);

        final var result16 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result16);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result17 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result17);

        final var result18 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result18);

        final var result19 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final var result20 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final var result21 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result21);

        final var result22 = new ECIFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result22);

        final var result23 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final var result24 = new ECIFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final var result25 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final var result26 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final var result27 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result27);

        final var result28 = new ECIFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result28);

        final var result29 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result29);

        final var result30 = new ECIFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result30);

        final var result31 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result31);

        final var result32 = new ECIFrame();
        navigator.navigate(timeInterval, oldFrame, kinematics, result32);

        final var result33 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final var result34 = new ECIFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ, result34);

        final var result35 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result35);

        final var result36 = new ECIFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final var result37 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final var result38 = new ECIFrame();
        navigator.navigate(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result38);

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
    }

    @Test
    void testNavigateAndReturnNew() throws InvalidRotationMatrixException, 
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final var navigator = new ECIInertialNavigator();

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
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

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
        final var result2 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result3 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result4 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result5 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result6 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result7 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var result8 = navigator.navigateAndReturnNew(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result9 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result10 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result11 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var result12 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result13 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result14 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result15 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result16 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result17 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final var result18 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result19 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result20 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result21 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result22 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result23 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result24 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result25 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result26 = navigator.navigateAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result27 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result28 = navigator.navigateAndReturnNew(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, 
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result29 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result30 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result31 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final var result32 = navigator.navigateAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result33 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result34 = navigator.navigateAndReturnNew(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result35 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result36 = navigator.navigateAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result37 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result38 = navigator.navigateAndReturnNew(timeInterval, oldFrame,
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
    }

    @Test
    void testNavigateEci() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
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
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

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

        final var result1 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final var result3 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result3);

        final var result4 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics, 
                result4);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result5 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final var result6 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final var result7 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result7);

        final var result8 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result9 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result9);

        final var result10 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result10);

        final var result11 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final var result12 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC, 
                oldVx, oldVy, oldVz, kinematics, result12);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result13 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result13);

        final var result14 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result14);

        final var result15 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result15);

        final var result16 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result16);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result17 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result17);

        final var result18 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ, result18);

        final var result19 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final var result20 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final var result21 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result21);

        final var result22 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result22);

        final var result23 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final var result24 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final var result25 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final var result26 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final var result27 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result27);

        final var result28 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result28);

        final var result29 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ, result29);

        final var result30 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result30);

        final var result31 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result31);

        final var result32 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldFrame, kinematics, result32);

        final var result33 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final var result34 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final var result35 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result35);

        final var result36 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldFrame, fx, fy, fz, 
                angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final var result37 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final var result38 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result38);

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
    }

    @Test
    void testNavigateEciAndReturnNew() throws InvalidRotationMatrixException, 
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
        final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

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

        final var result1 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result3 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final var result4 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final var oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final var result5 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result6 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result7 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final var result8 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldPosition, oldC, 
                oldVx, oldVy, oldVz, kinematics);

        final var oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final var oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final var oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final var result9 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result10 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result11 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var result12 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldVx, oldVy, oldVz, kinematics);

        final var oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final var result13 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result14 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var result15 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result16 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var result17 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result18 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularRateX, angularRateY, angularRateZ);

        final var result19 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result20 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result21 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result22 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, 
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result23 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result24 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result25 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result26 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC, 
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ, 
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result27 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result28 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final var result29 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final var result30 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz, 
                angularRateX, angularRateY, angularRateZ);

        final var result31 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final var result32 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldFrame, kinematics);

        final var result33 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result34 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldFrame, 
                accelerationX, accelerationY, accelerationZ, angularRateX, angularRateY, angularRateZ);

        final var result35 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result36 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result37 = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final var result38 = ECIInertialNavigator.navigateECIAndReturnNew(timeInterval, oldFrame,
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
    }

    @Test
    void testCompareNavigations() throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException,
            WrongSizeException, InertialNavigatorException {

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
            final var oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                    oldEcefFrame);
            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldEcefFrame);
            final var g = gravity.asMatrix();
            final var cbe = oldEcefFrame.getCoordinateTransformation().getMatrix();
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

            final var newFrame = ECIInertialNavigator.navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                    kinematics);

            final var newEcefFrame1 = ECEFInertialNavigator.navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                    oldEcefFrame, kinematics);
            final var newEcefFrame2 = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                    newFrame);

            final var newNedFrame1 = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newEcefFrame1);
            final var newNedFrame2 = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(newEcefFrame2);

            // compare
            assertTrue(newNedFrame1.equals(newNedFrame2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(TIMES, numValid);
    }
}
