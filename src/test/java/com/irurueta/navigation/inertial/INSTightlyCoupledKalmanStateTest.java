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
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class INSTightlyCoupledKalmanStateTest {

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double THRESHOLD = 1e-8;

    @Test
    void testConstructor() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {

        // test empty constructor
        var state = new INSTightlyCoupledKalmanState();

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
        assertEquals(0.0, state.getReceiverClockOffset(), 0.0);
        assertEquals(0.0, state.getReceiverClockDrift(), 0.0);
        assertNull(state.getCovariance());

        // test constructor with values
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS, 
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z, 
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor with measurement values
        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);
        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        state = new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset, receiverClockDrift, m));

        // test constructor with point position
        final var position = new InhomogeneousPoint3D(x, y, z);

        state = new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ, position,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ,
                position, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor with ECEF velocity and position
        final var ecefVelocity = new ECEFVelocity(vx, vy, vz);
        final var ecefPosition = new ECEFPosition(x, y, z);

        state = new INSTightlyCoupledKalmanState(c, ecefVelocity, ecefPosition, 
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, ecefVelocity,
                ecefPosition, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor with ECEF position and velocity
        final var positionAndVelocity = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);

        state = new INSTightlyCoupledKalmanState(c, positionAndVelocity, 
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor with frame
        final var frame = new ECEFFrame(ecefPosition, ecefVelocity, c);

        state = new INSTightlyCoupledKalmanState(frame, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(frame, accelerationBiasX,
                accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset,
                receiverClockDrift, m));

        // test constructor with measurements
        final var accelerationX = new Acceleration(accelerationBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(accelerationBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(accelerationBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var gyroX = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var gyroY = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var gyroZ = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var clockOffset = new Distance(receiverClockOffset, DistanceUnit.METER);
        final var clockDrift = new Speed(receiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        state = new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ,
                clockOffset, clockDrift, m));

        // test constructor with point
        state = new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ, position, 
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift,
                m));

        // test constructor with velocity and position
        state = new INSTightlyCoupledKalmanState(c, ecefVelocity, ecefPosition, 
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, ecefVelocity,
                ecefPosition, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift,
                m));

        // test constructor with ECEF position and velocity
        state = new INSTightlyCoupledKalmanState(c, positionAndVelocity, accelerationX, accelerationY, accelerationZ,
                gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(c, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test constructor with frame
        state = new INSTightlyCoupledKalmanState(frame, accelerationX, accelerationY, accelerationZ, 
                gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(frame, accelerationX,
                accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException

        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset, receiverClockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix,
                speedX, speedY, speedZ, position, accelerationBiasX, accelerationBiasY, accelerationBiasZ,
                gyroBiasX, gyroBiasY, gyroBiasZ, receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, speedX, speedY, speedZ,
                position, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, position,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, ecefVelocity,
                ecefPosition, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ,
                clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, speedX, speedY, speedZ,
                distanceX, distanceY, distanceZ, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ,
                clockOffset, clockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, distanceX, distanceY, distanceZ,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift,
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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, speedX, speedY, speedZ,
                position, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift,
                covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, speedX, speedY, speedZ, position,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, ecefVelocity,
                ecefPosition, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift,
                covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, ecefVelocity, ecefPosition,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance);

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
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
        assertSame(covariance, state.getCovariance());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(m, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, covariance));
        assertThrows(IllegalArgumentException.class, () -> new INSTightlyCoupledKalmanState(
                bodyToEcefCoordinateTransformationMatrix, positionAndVelocity,
                accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ, clockOffset, clockDrift, m));

        // test copy constructor
        state = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

        final var state2 = new INSTightlyCoupledKalmanState(state);

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
        assertEquals(receiverClockOffset, state2.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state2.getReceiverClockDrift(), 0.0);
        assertEquals(covariance, state2.getCovariance());
    }

    @Test
    void testGetSetBodyToEcefCoordinateTransformationMatrix() throws WrongSizeException {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());

        // set a new value
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();

        state.setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);

        // check
        assertSame(state.getBodyToEcefCoordinateTransformationMatrix(), bodyToEcefCoordinateTransformationMatrix);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> state.setBodyToEcefCoordinateTransformationMatrix(m1));
        final var m2 = new Matrix(CoordinateTransformation.ROWS, 1);
        assertThrows(IllegalArgumentException.class, () -> state.setBodyToEcefCoordinateTransformationMatrix(m2));
    }

    @Test
    void testGetSetVx() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVx(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVx(vx);

        // check
        assertEquals(vx, state.getVx(), 0.0);
    }

    @Test
    void testGetSetVy() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVy(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVy(vy);

        // check
        assertEquals(vy, state.getVy(), 0.0);
    }

    @Test
    void testGetSetVz() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getVz(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVz(vz);

        // check
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    void testSetVelocityCoordinates() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getVx(), 0.0);
        assertEquals(0.0, state.getVy(), 0.0);
        assertEquals(0.0, state.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setVelocityCoordinates(vx, vy, vz);

        // check
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    void testGetSetX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setX(x);

        // check
        assertEquals(x, state.getX(), 0.0);
    }

    @Test
    void testGetSetY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setY(y);

        // check
        assertEquals(y, state.getY(), 0.0);
    }

    @Test
    void testGetSetZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setZ(z);

        // check
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    void testSetPositionCoordinates() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getX(), 0.0);
        assertEquals(0.0, state.getY(), 0.0);
        assertEquals(0.0, state.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setPositionCoordinates(x, y, z);

        // check
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasX(accelerationBiasX);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasY(accelerationBiasY);

        // check
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasZ(accelerationBiasZ);

        // check
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testSetAccelerationBiasCoordinates() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetGyroBiasX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasX(gyroBiasX);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
    }

    @Test
    void testGetSetGyroBiasY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasY(gyroBiasY);

        // check
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
    }

    @Test
    void testGetSetGyroBiasZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasZ(gyroBiasZ);

        // check
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    void testSetGyroBiasCoordinates() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getGyroBiasX(), 0.0);
        assertEquals(0.0, state.getGyroBiasY(), 0.0);
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    void testGetSetReceiverClockOffset() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getReceiverClockOffset(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setReceiverClockOffset(receiverClockOffset);

        // check
        assertEquals(receiverClockOffset, state.getReceiverClockOffset(), 0.0);
    }

    @Test
    void testGetSetReceiverClockDrift() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getReceiverClockDrift(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        state.setReceiverClockDrift(receiverClockDrift);

        // check
        assertEquals(receiverClockDrift, state.getReceiverClockDrift(), 0.0);
    }

    @Test
    void testGetSetCovariance() throws WrongSizeException {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertFalse(state.getCovariance(null));
        assertNull(state.getCovariance());

        // set a new value
        final var covariance1 = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS, 
                INSTightlyCoupledKalmanState.NUM_PARAMS);
        state.setCovariance(covariance1);

        // check
        final var covariance2 = new Matrix(1, 1);
        assertTrue(state.getCovariance(covariance2));
        final var covariance3 = state.getCovariance();

        assertEquals(covariance1, covariance2);
        assertSame(covariance1, covariance3);
    }

    @Test
    void testGetSetC() throws InvalidRotationMatrixException {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertNull(state.getC());
        assertNull(state.getC(THRESHOLD));

        final var c1 = new CoordinateTransformation(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, 
                FrameType.EARTH_CENTERED_INERTIAL_FRAME);
        assertFalse(state.getC(c1));
        assertFalse(state.getC(c1, THRESHOLD));
        assertNull(state.getBodyToEcefCoordinateTransformationMatrix());

        // set a new value
        final var randomizer = new UniformRandomizer();

        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c2 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        state.setC(c2);

        // check
        final var c3 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(state.getC(c3));

        final var c4 = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        assertTrue(state.getC(c4, THRESHOLD));

        assertEquals(FrameType.BODY_FRAME, c3.getSourceType());
        assertEquals(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, c3.getDestinationType());
        assertEquals(c3.getMatrix(), c2.getMatrix());
        assertEquals(c2.getMatrix(), state.getBodyToEcefCoordinateTransformationMatrix());

        assertEquals(c2, c3);
        assertEquals(c2, c4);

        // set again
        final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c5 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        state.setC(c5);

        assertEquals(c5, state.getC());
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
    void testGetSetSpeedX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var speedX1 = state.getSpeedX();

        assertEquals(0.0, speedX1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedX1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var speedX2 = new Speed(vx, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedX(speedX2);

        // check
        final var speedX3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedX(speedX3);
        final var speedX4 = state.getSpeedX();

        assertEquals(speedX2, speedX3);
        assertEquals(speedX2, speedX4);
    }

    @Test
    void testGetSetSpeedY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var speedY1 = state.getSpeedY();

        assertEquals(0.0, speedY1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedY1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var speedY2 = new Speed(vy, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedY(speedY2);

        // check
        final var speedY3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedY(speedY3);
        final var speedY4 = state.getSpeedY();

        assertEquals(speedY2, speedY3);
        assertEquals(speedY2, speedY4);
    }

    @Test
    void testGetSetSpeedZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var speedZ1 = state.getSpeedZ();

        assertEquals(0.0, speedZ1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedZ1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var speedZ2 = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setSpeedZ(speedZ2);

        // check
        final var speedZ3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        state.getSpeedZ(speedZ3);
        final var speedZ4 = state.getSpeedZ();

        assertEquals(speedZ2, speedZ3);
        assertEquals(speedZ2, speedZ4);
    }

    @Test
    void testSetVelocityCoordinates2() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getVx(), 0.0);
        assertEquals(0.0, state.getVy(), 0.0);
        assertEquals(0.0, state.getVz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final var speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final var speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        state.setVelocityCoordinates(speedX, speedY, speedZ);

        // check
        assertEquals(vx, state.getVx(), 0.0);
        assertEquals(vy, state.getVy(), 0.0);
        assertEquals(vz, state.getVz(), 0.0);
    }

    @Test
    void testGetSetEcefVelocity() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var velocity1 = state.getEcefVelocity();

        assertEquals(new ECEFVelocity(), velocity1);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var velocity2 = new ECEFVelocity(vx, vy, vz);
        state.setEcefVelocity(velocity2);

        // check
        final var velocity3 = new ECEFVelocity();
        state.getEcefVelocity(velocity3);
        final var velocity4 = state.getEcefVelocity();

        assertEquals(velocity2, velocity3);
        assertEquals(velocity2, velocity4);
    }

    @Test
    void testGetSetDistanceX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var distanceX1 = state.getDistanceX();

        assertEquals(0.0, distanceX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceX1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var distanceX2 = new Distance(x, DistanceUnit.METER);

        state.setDistanceX(distanceX2);

        // check
        final var distanceX3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceX(distanceX3);
        final var distanceX4 = state.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    void testGetSetDistanceY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var distanceY1 = state.getDistanceY();

        assertEquals(0.0, distanceY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceY1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var distanceY2 = new Distance(y, DistanceUnit.METER);

        state.setDistanceY(distanceY2);

        // check
        final var distanceY3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceY(distanceY3);
        final var distanceY4 = state.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    void testGetSetDistanceZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var distanceZ1 = state.getDistanceZ();

        assertEquals(0.0, distanceZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distanceZ1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var distanceZ2 = new Distance(z, DistanceUnit.METER);

        state.setDistanceZ(distanceZ2);

        // check
        final var distanceZ3 = new Distance(0.0, DistanceUnit.METER);
        state.getDistanceZ(distanceZ3);
        final var distanceZ4 = state.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    void testSetPositionCoordinates2() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getX(), 0.0);
        assertEquals(0.0, state.getY(), 0.0);
        assertEquals(0.0, state.getZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        state.setPositionCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(x, state.getX(), 0.0);
        assertEquals(y, state.getY(), 0.0);
        assertEquals(z, state.getZ(), 0.0);
    }

    @Test
    void testGetSetPosition() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var position1 = state.getPosition();

        assertEquals(position1, Point3D.create());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var position2 = new InhomogeneousPoint3D(x, y, z);
        state.setPosition(position2);

        // check
        final var position3 = new InhomogeneousPoint3D();
        state.getPosition(position3);
        final var position4 = state.getPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetEcefPosition() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var position1 = state.getEcefPosition();

        assertEquals(new ECEFPosition(), position1);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var position2 = new ECEFPosition(x, y, z);
        state.setEcefPosition(position2);

        // check
        final var position3 = new ECEFPosition();
        state.getEcefPosition(position3);
        final var position4 = state.getEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetPositionAndVelocity() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var positionAndVelocity1 = state.getPositionAndVelocity();

        assertEquals(new ECEFPositionAndVelocity(), positionAndVelocity1);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var positionAndVelocity2 = new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
        state.setPositionAndVelocity(positionAndVelocity2);

        // check
        final var positionAndVelocity3 = new ECEFPositionAndVelocity();
        state.getPositionAndVelocity(positionAndVelocity3);
        final var positionAndVelocity4 = state.getPositionAndVelocity();

        assertEquals(positionAndVelocity2, positionAndVelocity3);
        assertEquals(positionAndVelocity2, positionAndVelocity4);
    }

    @Test
    void testGetSetFrame() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertFalse(state.getFrame(null));
        assertNull(state.getFrame());

        // set a new value
        final var randomizer = new UniformRandomizer();

        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var frame1 = new ECEFFrame(x1, y1, z1, vx1, vy1, vz1, c1);
        state.setFrame(frame1);

        // check
        final var frame2 = new ECEFFrame();
        assertTrue(state.getFrame(frame2));
        final var frame3 = state.getFrame();

        assertEquals(frame1, frame2);
        assertEquals(frame1, frame3);

        // set invalid transformation matrix with the correct size
        state.setBodyToEcefCoordinateTransformationMatrix(new Matrix(CoordinateTransformation.ROWS, 
                CoordinateTransformation.COLS));

        // check again
        assertFalse(state.getFrame(frame2));
        assertNull(state.getFrame());

        // set a new frame
        final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME, 
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final var x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var frame4 = new ECEFFrame(x2, y2, z2, vx2, vy2, vz2, c2);
        state.setFrame(frame4);

        assertEquals(frame4, state.getFrame());
    }

    @Test
    void testGetSetAccelerationBiasXAsAcceleration() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var accelerationX1 = state.getAccelerationBiasXAsAcceleration();

        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerationX2 = new Acceleration(accelerationX, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasX(accelerationX2);

        // check
        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasXAsAcceleration(accelerationX3);
        final var accelerationX4 = state.getAccelerationBiasXAsAcceleration();

        assertEquals(accelerationX2, accelerationX3);
        assertEquals(accelerationX2, accelerationX4);
    }

    @Test
    void testGetSetAccelerationBiasYAsAcceleration() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var accelerationY1 = state.getAccelerationBiasYAsAcceleration();

        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerationY2 = new Acceleration(accelerationY, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasY(accelerationY2);

        // check
        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasYAsAcceleration(accelerationY3);
        final var accelerationY4 = state.getAccelerationBiasYAsAcceleration();

        assertEquals(accelerationY2, accelerationY3);
        assertEquals(accelerationY2, accelerationY4);
    }

    @Test
    void testGetSetAccelerationBiasZAsAcceleration() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var accelerationZ1 = state.getAccelerationBiasZAsAcceleration();

        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerationZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerationZ2 = new Acceleration(accelerationZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasZ(accelerationZ2);

        // check
        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        state.getAccelerationBiasZAsAcceleration(accelerationZ3);
        final var accelerationZ4 = state.getAccelerationBiasZAsAcceleration();

        assertEquals(accelerationZ2, accelerationZ3);
        assertEquals(accelerationZ2, accelerationZ4);
    }

    @Test
    void testSetAccelerationBiasCoordinates2() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default values
        assertEquals(0.0, state.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, state.getAccelerationBiasZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerationX = new Acceleration(accelerationBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationY = new Acceleration(accelerationBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var accelerationZ = new Acceleration(accelerationBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        state.setAccelerationBiasCoordinates(accelerationX, accelerationY, accelerationZ);

        // check
        assertEquals(accelerationBiasX, state.getAccelerationBiasX(), 0.0);
        assertEquals(accelerationBiasY, state.getAccelerationBiasY(), 0.0);
        assertEquals(accelerationBiasZ, state.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedGyroBiasX() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var angularSpeedX1 = state.getAngularSpeedGyroBiasX();

        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var angularSpeedX2 = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasX(angularSpeedX2);

        // check
        final var angularSpeedX3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasX(angularSpeedX3);
        final var angularSpeedX4 = state.getAngularSpeedGyroBiasX();

        assertEquals(angularSpeedX2, angularSpeedX3);
        assertEquals(angularSpeedX2, angularSpeedX4);
    }

    @Test
    void testGetSetAngularSpeedGyroBiasY() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var angularSpeedY1 = state.getAngularSpeedGyroBiasY();

        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var angularSpeedY2 = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasY(angularSpeedY2);

        // check
        final var angularSpeedY3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasY(angularSpeedY3);
        final var angularSpeedY4 = state.getAngularSpeedGyroBiasY();

        assertEquals(angularSpeedY2, angularSpeedY3);
        assertEquals(angularSpeedY2, angularSpeedY4);
    }

    @Test
    void testGetSetAngularSpeedGyroBiasZ() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var angularSpeedZ1 = state.getAngularSpeedGyroBiasZ();

        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var angularSpeedZ2 = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasZ(angularSpeedZ2);

        // check
        final var angularSpeedZ3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        state.getAngularSpeedGyroBiasZ(angularSpeedZ3);
        final var angularSpeedZ4 = state.getAngularSpeedGyroBiasZ();

        assertEquals(angularSpeedZ2, angularSpeedZ3);
        assertEquals(angularSpeedZ2, angularSpeedZ4);
    }

    @Test
    void testSetGyroBiasCoordinates2() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        assertEquals(0.0, state.getGyroBiasX(), 0.0);
        assertEquals(0.0, state.getGyroBiasY(), 0.0);
        assertEquals(0.0, state.getGyroBiasZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var angularSpeedX = new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        state.setGyroBiasCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check
        assertEquals(gyroBiasX, state.getGyroBiasX(), 0.0);
        assertEquals(gyroBiasY, state.getGyroBiasY(), 0.0);
        assertEquals(gyroBiasZ, state.getGyroBiasZ(), 0.0);
    }

    @Test
    void testGetSetReceiverClockOffsetAsDistance() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var clockOffset1 = state.getReceiverClockOffsetAsDistance();

        assertEquals(0.0, clockOffset1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, clockOffset1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var clockOffset2 = new Distance(receiverClockOffset, DistanceUnit.METER);

        state.setReceiverClockOffset(clockOffset2);

        // check
        final var clockOffset3 = new Distance(0.0, DistanceUnit.METER);
        state.getReceiverClockOffsetAsDistance(clockOffset3);
        final var clockOffset4 = state.getReceiverClockOffsetAsDistance();

        assertEquals(clockOffset2, clockOffset3);
        assertEquals(clockOffset2, clockOffset4);
    }

    @Test
    void testGetSetReceiverClockDriftAsSpeed() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var clockDrift1 = state.getReceiverClockDriftAsSpeed();

        assertEquals(0.0, clockDrift1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, clockDrift1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var clockDrift2 = new Speed(receiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        state.setReceiverClockDrift(clockDrift2);

        // check
        final var clockDrift3 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        state.getReceiverClockDriftAsSpeed(clockDrift3);
        final var clockDrift4 = state.getReceiverClockDriftAsSpeed();

        assertEquals(clockDrift2, clockDrift3);
        assertEquals(clockDrift2, clockDrift4);
    }

    @Test
    void testGetSetGNSSEstimation() {
        final var state = new INSTightlyCoupledKalmanState();

        // check default value
        final var estimation1 = state.getGNSSEstimation();

        assertEquals(0.0, estimation1.getX(), 0.0);
        assertEquals(0.0, estimation1.getY(), 0.0);
        assertEquals(0.0, estimation1.getZ(), 0.0);

        assertEquals(0.0, estimation1.getVx(), 0.0);
        assertEquals(0.0, estimation1.getVy(), 0.0);
        assertEquals(0.0, estimation1.getVz(), 0.0);

        assertEquals(0.0, estimation1.getClockOffset(), 0.0);
        assertEquals(0.0, estimation1.getClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var estimation2 = new GNSSEstimation(x, y, z, vx, vy, vz, receiverClockOffset, receiverClockDrift);
        state.setGNSSEstimation(estimation2);

        // check
        final var estimation3 = new GNSSEstimation();
        state.getGNSSEstimation(estimation3);
        final var estimation4 = state.getGNSSEstimation();

        assertEquals(estimation2, estimation3);
        assertEquals(estimation2, estimation4);
    }

    @Test
    void testCopyToWhenInputHasValuesAndOutputDoesNotHaveValues() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

        final var state2 = new INSTightlyCoupledKalmanState();
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
        assertEquals(receiverClockOffset, state2.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift, state2.getReceiverClockDrift(), 0.0);
        assertEquals(covariance, state2.getCovariance());
    }

    @Test
    void testCopyToWhenInputHasNoValuesAndOutputHasValues() throws WrongSizeException {
        final var state1 = new INSTightlyCoupledKalmanState();

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
        assertEquals(0.0, state1.getReceiverClockOffset(), 0.0);
        assertEquals(0.0, state1.getReceiverClockDrift(), 0.0);
        assertNull(state1.getCovariance());

        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

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
        assertEquals(0.0, state2.getReceiverClockOffset(), 0.0);
        assertEquals(0.0, state2.getReceiverClockDrift(), 0.0);
        assertNull(state2.getCovariance());
    }

    @Test
    void testCopyToWhenInputHasNoValuesAndOutputDoesNotHaveValues() {
        final var state1 = new INSTightlyCoupledKalmanState();

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
        assertEquals(0.0, state1.getReceiverClockOffset(), 0.0);
        assertEquals(0.0, state1.getReceiverClockDrift(), 0.0);
        assertNull(state1.getCovariance());

        final var state2 = new INSTightlyCoupledKalmanState();
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
        assertEquals(0.0, state2.getReceiverClockOffset(), 0.0);
        assertEquals(0.0, state2.getReceiverClockDrift(), 0.0);
        assertNull(state2.getCovariance());
    }

    @Test
    void testCopyToWhenBothInputAndOutputHaveValues() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix1 = c1.getMatrix();
        final var vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance1 = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix1, vx1, vy1, vz1,
                x1, y1, z1, accelerationBiasX1, accelerationBiasY1, accelerationBiasZ1,
                gyroBiasX1, gyroBiasY1, gyroBiasZ1, receiverClockOffset1, receiverClockDrift1, covariance1);


        final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix2 = c2.getMatrix();
        final var vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance2 = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix2, vx2, vy2, vz2,
                x2, y2, z2, accelerationBiasX2, accelerationBiasY2, accelerationBiasZ2,
                gyroBiasX2, gyroBiasY2, gyroBiasZ2, receiverClockOffset2, receiverClockDrift2, covariance2);

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
        assertEquals(receiverClockOffset1, state2.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift1, state2.getReceiverClockDrift(), 0.0);
        assertEquals(covariance1, state2.getCovariance());
    }

    @Test
    void testCopyFrom() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c1 = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix1 = c1.getMatrix();
        final var vx1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift1 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance1 = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix1, vx1, vy1, vz1,
                x1, y1, z1, accelerationBiasX1, accelerationBiasY1, accelerationBiasZ1,
                gyroBiasX1, gyroBiasY1, gyroBiasZ1, receiverClockOffset1, receiverClockDrift1, covariance1);

        final var roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c2 = new CoordinateTransformation(roll2, pitch2, yaw2, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix2 = c2.getMatrix();
        final var vx2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift2 = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance2 = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix2, vx2, vy2, vz2,
                x2, y2, z2, accelerationBiasX2, accelerationBiasY2, accelerationBiasZ2,
                gyroBiasX2, gyroBiasY2, gyroBiasZ2, receiverClockOffset2, receiverClockDrift2, covariance2);

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
        assertEquals(receiverClockOffset1, state2.getReceiverClockOffset(), 0.0);
        assertEquals(receiverClockDrift1, state2.getReceiverClockDrift(), 0.0);
        assertEquals(covariance1, state2.getCovariance());
    }

    @Test
    void testHashCode() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state3 = new INSTightlyCoupledKalmanState();

        assertEquals(state1.hashCode(), state2.hashCode());
        assertNotEquals(state1.hashCode(), state3.hashCode());
    }

    @Test
    void testEquals() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state3 = new INSTightlyCoupledKalmanState();

        //noinspection EqualsWithItself
        assertEquals(state1, state1);
        //noinspection EqualsWithItself
        assertTrue(state1.equals(state1));
        assertTrue(state1.equals(state2));
        assertFalse(state1.equals(state3));
        assertNotEquals(null, state1);
        assertFalse(state1.equals(null));
        assertNotEquals(new Object(), state1);
    }

    @Test
    void testEqualsWithThreshold() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state2 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);
        final var state3 = new INSTightlyCoupledKalmanState();

        assertTrue(state1.equals(state1, THRESHOLD));
        assertTrue(state1.equals(state2, THRESHOLD));
        assertFalse(state1.equals(state3, THRESHOLD));
        assertFalse(state1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws WrongSizeException, CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

        final var state2 = state1.clone();

        assertEquals(state1, state2);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        final var bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
        final var vx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var vz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS,
                INSTightlyCoupledKalmanState.NUM_PARAMS);

        final var state1 = new INSTightlyCoupledKalmanState(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz,
                x, y, z, accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                receiverClockOffset, receiverClockDrift, covariance);

        final var bytes = SerializationHelper.serialize(state1);
        final var state2 = SerializationHelper.deserialize(bytes);

        assertEquals(state1, state2);
        assertNotSame(state1, state2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = INSTightlyCoupledKalmanState.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
