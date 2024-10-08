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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class INSLooselyCoupledKalmanStateTest {

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double THRESHOLD = 1e-8;

    @Test
    public void testConstructor() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(0.0, state.getVx(), 0.0);
        assertEquals(0.0, state.getVy(), 0.0);
        assertEquals(0.0, state.getVz(), 0.0);
        assertEquals(0.0, state.getX(), 0.0);
        assertEquals(0.0, state.getY(), 0.0);
        assertEquals(0.0, state.getZ(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);
        assertEquals(0.0, state.getGyroBiasX(), 0.0);
        assertEquals(0.0, state.getGyroBiasY(), 0.0);
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);
        assertNull(state.getCovariance());

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertSame(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor with measurement values
        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        state = new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor with point position
        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        state = new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ, position,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ,
                position, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                m));

        // test constructor with ECEF velocity and position
        final ECEFVelocity ecefVelocity = new ECEFVelocity(vx, vy, vz);
        final ECEFPosition ecefPosition = new ECEFPosition(x, y, z);

        state = new INSLooselyCoupledKalmanState(c, ecefVelocity, ecefPosition,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, ecefVelocity,
                ecefPosition, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                m));

        // test constructor with ECEF position and velocity
        final ECEFPositionAndVelocity positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        state = new INSLooselyCoupledKalmanState(c, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor with frame
        final ECEFFrame frame = new ECEFFrame(ecefPosition, ecefVelocity, c);

        state = new INSLooselyCoupledKalmanState(frame, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(frame, accelerationBiasX,
                accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor with measurements
        final Acceleration accelerationX = new Acceleration(accelerationBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(accelerationBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(accelerationBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final AngularSpeed gyroX = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed gyroY = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed gyroZ = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        state = new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor with point
        state = new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ, position,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor with velocity and position
        state = new INSLooselyCoupledKalmanState(c, ecefVelocity, ecefPosition,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, ecefVelocity,
                ecefPosition, accelerationX, accelerationY, accelerationZ,
                gyroX, gyroY, gyroZ, m));

        // test constructor with ECEF position and velocity
        state = new INSLooselyCoupledKalmanState(c, positionAndVelocity, accelerationX, accelerationY, accelerationZ,
                gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(c, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor with frame
        state = new INSLooselyCoupledKalmanState(frame, accelerationX, accelerationY, accelerationZ,
                gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(frame,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix,
                speedX, speedY, speedZ, position, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, speedX, speedY, speedZ,
                position, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, position,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, ecefVelocity,
                ecefPosition, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, ecefVelocity,
                ecefPosition, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ,
                covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ,
                covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, position,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, ecefVelocity,
                ecefPosition, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, ecefVelocity,
                ecefPosition, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(m, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, m));

        // test copy constructor
        state = new INSLooselyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(state);

        // check default values
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state2.getVx(), 0.0);
        assertEquals(vy, state2.getVy(), 0.0);
        assertEquals(vz, state2.getVz(), 0.0);
        assertEquals(x, state2.getX(), 0.0);
        assertEquals(y, state2.getY(), 0.0);
        assertEquals(z, state2.getZ(), 0.0);
        assertEquals(accelerationBiasX, state2.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state2.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state2.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state2.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state2.getGyroBiasZ(), 0.0);
        assertEquals(covariance, state2.getCovariance());
    }

    @Test
    public void testGetSetBodyToEcefCoordinateTransformationMatrix() throws WrongSizeException {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();

        state.setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);

        // check
        assertSame(bodyToEcefCoordinateTransformationMatrix, state.getBodyToEcefCoordinateTransformationMatrix());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> state.setBodyToEcefCoordinateTransformationMatrix(m1));
        final var m2 = new Matrix(CoordinateTransformation.ROWS, 1);
        assertThrows(IllegalArgumentException.class, () -> state.setBodyToEcefCoordinateTransformationMatrix(m2));
    }

    @Test
    public void testGetSetVx() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVx(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVx(vx);

        // check
        assertEquals(vx, state.getVx(), 0.0);
    }

    @Test
    public void testGetSetVy() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVy(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVy(vy);

        // check
        assertEquals(vy, state.getVy(), 0.0);
    }

    @Test
    public void testGetSetVz() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVz(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVz(vz);

        // check
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    public void testSetVelocityCoordinates() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getVx(), 0.0);
        assertEquals(0.0, state.getVy(), 0.0);
        assertEquals(0.0, state.getVz(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    public void testGetSetX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setX(x);

        // check
        assertEquals(x, state.getX(), 0.0);
    }

    @Test
    public void testGetSetY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setY(y);

        // check
        assertEquals(y, state.getY(), 0.0);
    }

    @Test
    public void testGetSetZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setZ(z);

        // check
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    public void testSetPositionCoordinates() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getX(), 0.0);
        assertEquals(0.0, state.getY(), 0.0);
        assertEquals(0.0, state.getZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasX(accelerationBiasX);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasY(accelerationBiasY);

        // check
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasZ(accelerationBiasZ);

        // check
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testSetAccelerationBiasCoordinates() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetGyroBiasX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasX(gyroBiasX);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
    }

    @Test
    public void testGetSetGyroBiasY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasY(gyroBiasY);

        // check
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
    }

    @Test
    public void testGetSetGyroBiasZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasZ(gyroBiasZ);

        // check
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    public void testSetGyroBiasCoordinates() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getGyroBiasX(), 0.0);
        assertEquals(0.0, state.getGyroBiasY(), 0.0);
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertFalse(state.getCovariance(null));
        assertNull(state.getCovariance());

        // set new value
        final Matrix covariance1 = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);
        state.setCovariance(covariance1);

        // check
        final Matrix covariance2 = new Matrix(1, 1);
        assertTrue(state.getCovariance(covariance2));
        final Matrix covariance3 = state.getCovariance();

        assertEquals(covariance1, covariance2);
        assertSame(covariance1, covariance3);
    }

    @Test
    public void testGetSetC() throws InvalidRotationMatrixException {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertNull(state.getC());
        assertNull(state.getC(THRESHOLD));

        final CoordinateTransformation c1 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        assertFalse(state.getC(c1));
        assertFalse(state.getC(c1, THRESHOLD));
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c2 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        state.setC(c2);

        // check
        final CoordinateTransformation c3 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(state.getC(c3));

        final CoordinateTransformation c4 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(state.getC(c4, THRESHOLD));

        assertEquals(FrameType.BODY_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), c2.getMatrix());
        assertEquals(c2.getMatrix(), state.getBodyToEcefCoordinateTransformationMatrix());

        assertEquals(c2, c3);
        assertEquals(c2, c4);

        // set again
        final double roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c5 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        state.setC(c5);

        assertTrue(c5.equals(state.getC(), THRESHOLD));
        assertEquals(c5.getMatrix(), state.getBodyToEcefCoordinateTransformationMatrix());

        state.setC(null);

        assertNull(state.getC());
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());

        // Force IllegalArgumentException
        final var c = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        assertThrows(IllegalArgumentException.class, () -> state.setC(c));
    }

    @Test
    public void testGetSetSpeedX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Speed speedX1 = state.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedX(speedX2);

        // check
        final Speed speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedX(speedX3);
        final Speed speedX4 = state.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    public void testGetSetSpeedY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Speed speedY1 = state.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedY(speedY2);

        // check
        final Speed speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedY(speedY3);
        final Speed speedY4 = state.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    public void testGetSetSpeedZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Speed speedZ1 = state.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedZ(speedZ2);

        // check
        final Speed speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedZ(speedZ3);
        final Speed speedZ4 = state.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    public void testSetVelocityCoordinates2() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getVx(), 0.0);
        assertEquals(0.0, state.getVy(), 0.0);
        assertEquals(0.0, state.getVz(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setVelocityCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    public void testGetSetEcefVelocity() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final ECEFVelocity velocity1 = state.getEcefVelocity();

        assertEquals(new ECEFVelocity(), velocity1);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFVelocity velocity2 = new ECEFVelocity(vx, vy, vz);
        state.setEcefVelocity(velocity2);

        // check
        final ECEFVelocity velocity3 = new ECEFVelocity();
        state.getEcefVelocity(velocity3);
        final ECEFVelocity velocity4 = state.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    public void testGetSetDistanceX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Distance distanceX1 = state.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);

        state.setDistanceX(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceX(distanceX3);
        final Distance distanceX4 = state.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetDistanceY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Distance distanceY1 = state.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);

        state.setDistanceY(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceY(distanceY3);
        final Distance distanceY4 = state.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetDistanceZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Distance distanceZ1 = state.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        state.setDistanceZ(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceZ(distanceZ3);
        final Distance distanceZ4 = state.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetPositionCoordinates2() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getX(), 0.0);
        assertEquals(0.0, state.getY(), 0.0);
        assertEquals(0.0, state.getZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        state.setPositionCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    public void testGetSetPosition() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Point3D position1 = state.getPosition();

        assertEquals(position1, Point3D.create());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        state.setPosition(position2);

        // check
        final Point3D position3 = new InhomogeneousPoint3D();
        state.getPosition(position3);
        final Point3D position4 = state.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetEcefPosition() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final ECEFPosition position1 = state.getEcefPosition();

        assertEquals(position1, new ECEFPosition());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        state.setEcefPosition(position2);

        // check
        final ECEFPosition position3 = new ECEFPosition();
        state.getEcefPosition(position3);
        final ECEFPosition position4 = state.getEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetPositionAndVelocity() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final ECEFPositionAndVelocity positionAndVelocity1 = state.getPositionAndVelocity();

        assertEquals(positionAndVelocity1, new ECEFPositionAndVelocity());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFPositionAndVelocity positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        state.setPositionAndVelocity(positionAndVelocity2);

        // check
        final ECEFPositionAndVelocity positionAndVelocity3 = new ECEFPositionAndVelocity();
        state.getPositionAndVelocity(positionAndVelocity3);
        final ECEFPositionAndVelocity positionAndVelocity4 = state.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    public void testGetSetFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertFalse(state.getFrame(null));
        assertNull(state.getFrame());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final double x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final double vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFFrame frame1 = new ECEFFrame(x1, y1, z1, vx1, vy1, vz1, c1);
        state.setFrame(frame1);

        // check
        final ECEFFrame frame2 = new ECEFFrame();
        assertTrue(state.getFrame(frame2));
        final ECEFFrame frame3 = state.getFrame();

        assertTrue(frame1.equals(frame2, THRESHOLD));
        assertTrue(frame1.equals(frame3, THRESHOLD));
        assertEquals(frame2, frame3);

        // set new frame
        final double roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final double x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final double vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final ECEFFrame frame4 = new ECEFFrame(x2, y2, z2, vx2, vy2, vz2, c2);
        state.setFrame(frame4);

        assertTrue(frame4.equals(state.getFrame(), THRESHOLD));
    }

    @Test
    public void testGetSetAccelerationBiasXAsAcceleration() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Acceleration accelerationX1 = state.getAccelerationBiasXAsAcceleration();

        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Acceleration accelerationX2 = new Acceleration(accelerationX, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasX(accelerationX2);

        // check
        final Acceleration accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasXAsAcceleration(accelerationX3);
        final Acceleration accelerationX4 = state.getAccelerationBiasXAsAcceleration();

        assertEquals(accelerationX2, accelerationX3);
        assertEquals(accelerationX2, accelerationX4);
    }

    @Test
    public void testGetSetAccelerationBiasYAsAcceleration() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Acceleration accelerationY1 = state.getAccelerationBiasYAsAcceleration();

        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Acceleration accelerationY2 = new Acceleration(accelerationY, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasY(accelerationY2);

        // check
        final Acceleration accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasYAsAcceleration(accelerationY3);
        final Acceleration accelerationY4 = state.getAccelerationBiasYAsAcceleration();

        assertEquals(accelerationY2, accelerationY3);
        assertEquals(accelerationY2, accelerationY4);
    }

    @Test
    public void testGetSetAccelerationBiasZAsAcceleration() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final Acceleration accelerationZ1 = state.getAccelerationBiasZAsAcceleration();

        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Acceleration accelerationZ2 = new Acceleration(accelerationZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasZ(accelerationZ2);

        // check
        final Acceleration accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasZAsAcceleration(accelerationZ3);
        final Acceleration accelerationZ4 = state.getAccelerationBiasZAsAcceleration();

        assertEquals(accelerationZ2, accelerationZ3);
        assertEquals(accelerationZ2, accelerationZ4);
    }

    @Test
    public void testSetAccelerationBiasCoordinates2() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Acceleration accelerationX = new Acceleration(accelerationBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(accelerationBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(accelerationBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasCoordinates(accelerationX, accelerationY, accelerationZ);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedGyroBiasX() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final AngularSpeed angularSpeedX1 = state.getAngularSpeedGyroBiasX();

        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final AngularSpeed angularSpeedX2 = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasX(angularSpeedX2);

        // check
        final AngularSpeed angularSpeedX3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasX(angularSpeedX3);
        final AngularSpeed angularSpeedX4 = state.getAngularSpeedGyroBiasX();

        assertEquals(angularSpeedX2, angularSpeedX3);
        assertEquals(angularSpeedX2, angularSpeedX4);
    }

    @Test
    public void testGetSetAngularSpeedGyroBiasY() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final AngularSpeed angularSpeedY1 = state.getAngularSpeedGyroBiasY();

        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final AngularSpeed angularSpeedY2 = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasY(angularSpeedY2);

        // check
        final AngularSpeed angularSpeedY3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasY(angularSpeedY3);
        final AngularSpeed angularSpeedY4 = state.getAngularSpeedGyroBiasY();

        assertEquals(angularSpeedY2, angularSpeedY3);
        assertEquals(angularSpeedY2, angularSpeedY4);
    }

    @Test
    public void testGetSetAngularSpeedGyroBiasZ() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        final AngularSpeed angularSpeedZ1 = state.getAngularSpeedGyroBiasZ();

        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final AngularSpeed angularSpeedZ2 = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasZ(angularSpeedZ2);

        // check
        final AngularSpeed angularSpeedZ3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasZ(angularSpeedZ3);
        final AngularSpeed angularSpeedZ4 = state.getAngularSpeedGyroBiasZ();

        assertEquals(angularSpeedZ2, angularSpeedZ3);
        assertEquals(angularSpeedZ2, angularSpeedZ4);
    }

    @Test
    public void testSetGyroBiasCoordinates2() {
        final INSLooselyCoupledKalmanState state = new INSLooselyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasX(), 0.0);
        assertEquals(0.0, state.getGyroBiasY(), 0.0);
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final AngularSpeed angularSpeedX = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    public void testCopyToWhenInputHasValuesAndOutputDoesNotHaveValues() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
        state1.copyTo(state2);

        // check
        assertEquals(bodyToEcefCoordinateTransformationMatrix, state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx, state2.getVx(), 0.0);
        assertEquals(vy, state2.getVy(), 0.0);
        assertEquals(vz, state2.getVz(), 0.0);
        assertEquals(x, state2.getX(), 0.0);
        assertEquals(y, state2.getY(), 0.0);
        assertEquals(z, state2.getZ(), 0.0);
        assertEquals(accelerationBiasX, state2.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state2.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX, state2.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state2.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state2.getGyroBiasZ(), 0.0);
        assertEquals(covariance, state2.getCovariance());
    }

    @Test
    public void testCopyToWhenInputHasNoValuesAndOutputHasValues() throws WrongSizeException {

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState();

        // check default values
        assertNull(state1.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(0.0, state1.getVx(), 0.0);
        assertEquals(0.0, state1.getVy(), 0.0);
        assertEquals(0.0, state1.getVz(), 0.0);
        assertEquals(0.0, state1.getX(), 0.0);
        assertEquals(0.0, state1.getY(), 0.0);
        assertEquals(0.0, state1.getZ(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasZ(), 0.0);
        assertEquals(0.0, state1.getGyroBiasX(), 0.0);
        assertEquals(0.0, state1.getGyroBiasY(), 0.0);
        assertEquals(0.0, state1.getGyroBiasZ(), 0.0);
        assertNull(state1.getCovariance());

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        state1.copyTo(state2);

        // check
        assertNull(state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(0.0, state2.getVx(), 0.0);
        assertEquals(0.0, state2.getVy(), 0.0);
        assertEquals(0.0, state2.getVz(), 0.0);
        assertEquals(0.0, state2.getX(), 0.0);
        assertEquals(0.0, state2.getY(), 0.0);
        assertEquals(0.0, state2.getZ(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(0.0, state2.getGyroBiasX(), 0.0);
        assertEquals(0.0, state2.getGyroBiasY(), 0.0);
        assertEquals(0.0, state2.getGyroBiasZ(), 0.0);
        assertNull(state2.getCovariance());
    }

    @Test
    public void testCopyToWhenInputHasNoValuesAndOutputDoesNotHaveValues() {
        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState();

        // check default values
        assertNull(state1.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(0.0, state1.getVx(), 0.0);
        assertEquals(0.0, state1.getVy(), 0.0);
        assertEquals(0.0, state1.getVz(), 0.0);
        assertEquals(0.0, state1.getX(), 0.0);
        assertEquals(0.0, state1.getY(), 0.0);
        assertEquals(0.0, state1.getZ(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state1.getAccelerationBiasZ(), 0.0);
        assertEquals(0.0, state1.getGyroBiasX(), 0.0);
        assertEquals(0.0, state1.getGyroBiasY(), 0.0);
        assertEquals(0.0, state1.getGyroBiasZ(), 0.0);
        assertNull(state1.getCovariance());

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState();
        state1.copyTo(state2);

        // check
        assertNull(state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(0.0, state2.getVx(), 0.0);
        assertEquals(0.0, state2.getVy(), 0.0);
        assertEquals(0.0, state2.getVz(), 0.0);
        assertEquals(0.0, state2.getX(), 0.0);
        assertEquals(0.0, state2.getY(), 0.0);
        assertEquals(0.0, state2.getZ(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(0.0, state2.getGyroBiasX(), 0.0);
        assertEquals(0.0, state2.getGyroBiasY(), 0.0);
        assertEquals(0.0, state2.getGyroBiasZ(), 0.0);
        assertNull(state2.getCovariance());
    }

    @Test
    public void testCopyToWhenBothInputAndOutputHaveValues() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix1 = c1.getMatrix();
        final double vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance1 = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix1, vx1, vy1, vz1, x1, y1, z1,
                accelerationBiasX1, accelerationBiasY1, accelerationBiasZ1, gyroBiasX1, gyroBiasY1, gyroBiasZ1,
                covariance1);

        final double roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix2 = c2.getMatrix();
        final double vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance2 = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix2, vx2, vy2, vz2, x2, y2, z2,
                accelerationBiasX2, accelerationBiasY2, accelerationBiasZ2, gyroBiasX2, gyroBiasY2, gyroBiasZ2,
                covariance2);

        state1.copyTo(state2);

        // check
        assertEquals(bodyToEcefCoordinateTransformationMatrix1, state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx1, state2.getVx(), 0.0);
        assertEquals(vy1, state2.getVy(), 0.0);
        assertEquals(vz1, state2.getVz(), 0.0);
        assertEquals(x1, state2.getX(), 0.0);
        assertEquals(y1, state2.getY(), 0.0);
        assertEquals(z1, state2.getZ(), 0.0);
        assertEquals(accelerationBiasX1, state2.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY1, state2.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ1, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX1, state2.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY1, state2.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ1, state2.getGyroBiasZ(), 0.0);
        assertEquals(covariance1, state2.getCovariance());
    }

    @Test
    public void testCopyFrom() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix1 = c1.getMatrix();
        final double vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance1 = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix1, vx1, vy1, vz1, x1, y1, z1,
                accelerationBiasX1, accelerationBiasY1, accelerationBiasZ1, gyroBiasX1, gyroBiasY1, gyroBiasZ1,
                covariance1);

        final double roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix2 = c2.getMatrix();
        final double vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance2 = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix2, vx2, vy2, vz2, x2, y2, z2,
                accelerationBiasX2, accelerationBiasY2, accelerationBiasZ2, gyroBiasX2, gyroBiasY2, gyroBiasZ2,
                covariance2);

        state2.copyFrom(state1);

        // check
        assertEquals(bodyToEcefCoordinateTransformationMatrix1, state2.getBodyToEcefCoordinateTransformationMatrix());
        assertEquals(vx1, state2.getVx(), 0.0);
        assertEquals(vy1, state2.getVy(), 0.0);
        assertEquals(vz1, state2.getVz(), 0.0);
        assertEquals(x1, state2.getX(), 0.0);
        assertEquals(y1, state2.getY(), 0.0);
        assertEquals(z1, state2.getZ(), 0.0);
        assertEquals(accelerationBiasX1, state2.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY1, state2.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ1, state2.getAccelerationBiasZ(), 0.0);
        assertEquals(gyroBiasX1, state2.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY1, state2.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ1, state2.getGyroBiasZ(), 0.0);
        assertEquals(covariance1, state2.getCovariance());
    }

    @Test
    public void testHashCode() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state3 = new INSLooselyCoupledKalmanState();

        assertEquals(state1.hashCode(), state2.hashCode());
        assertNotEquals(state1.hashCode(), state3.hashCode());
    }

    @Test
    public void testEquals() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw,
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state3 = new INSLooselyCoupledKalmanState();

        //noinspection EqualsWithItself
        assertEquals(state1, state1);
        //noinspection EqualsWithItself
        assertTrue(state1.equals(state1));
        assertTrue(state1.equals(state2));
        assertFalse(state1.equals(state3));
        assertNotEquals(state1, null);
        assertFalse(state1.equals(null));
        assertNotEquals(state1, new Object());
    }

    @Test
    public void testEqualsWithThreshold() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state2 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);
        final INSLooselyCoupledKalmanState state3 = new INSLooselyCoupledKalmanState();

        assertTrue(state1.equals(state1, THRESHOLD));
        assertTrue(state1.equals(state2, THRESHOLD));
        assertFalse(state1.equals(state3, THRESHOLD));
        assertFalse(state1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws WrongSizeException, CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        final Object state2 = state1.clone();

        assertEquals(state1, state2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, WrongSizeException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final Matrix bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final double vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Matrix covariance = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final INSLooselyCoupledKalmanState state1 = new INSLooselyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, covariance);

        final byte[] bytes = SerializationHelper.serialize(state1);
        final INSLooselyCoupledKalmanState state2 = SerializationHelper.deserialize(bytes);

        assertEquals(state1, state2);
        assertNotSame(state1, state2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = INSLooselyCoupledKalmanState.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
