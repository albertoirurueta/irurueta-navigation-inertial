/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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

import static org.junit.jupiter.api.Assertions.*;

import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import org.junit.jupiter.api.Test;

class ECIInertialNavigator2Test {
    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_SPECIFIC_FORCE = -12.0;
    private static final double MAX_SPECIFIC_FORCE = 12.0;

    private static final double MIN_ANGULAR_RATE_DEGREES_PER_SECOND = -5.0;
    private static final double MAX_ANGULAR_RATE_DEGREES_PER_SECOND = 5.0;

    private static final int TIMES = 100;

    @Test
    void testNavigateECiWhenInvalidCoordinateTransformationMatrix() {
        final var c = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final var result = new ECIFrame();
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class, () -> ECIInertialNavigator2.navigateECI(
                0.0, 0.0, 0.0, 0.0, c, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, result));
    }

    @Test
    void testNavigateECI() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {
        for (var t = 0; t < TIMES; t++) {
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
            final var oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, 
                    oldEcefFrame);

            final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
            final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
            final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

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

            final var result1 = new ECIFrame();
            final var result2 = new ECIFrame();
            ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result1);
            ECIInertialNavigator2.navigateECI(TIME_INTERVAL_SECONDS, oldFrame, kinematics, result2);

            assertEquals(result1, result2);
        }
    }
}