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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class BodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var k = new BodyKinematics();

        // check default values
        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);
        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        assertEquals(0.0, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        var triad1 = k.getSpecificForceTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(0.0, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        var triad2 = k.getAngularRateTriad();
        assertEquals(0.0, triad2.getValueX(), 0.0);
        assertEquals(0.0, triad2.getValueY(), 0.0);
        assertEquals(0.0, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        assertEquals(0.0, k.getSpecificForceNorm(), 0.0);
        assertEquals(0.0, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(0.0, k.getAngularRateNorm(), 0.0);
        assertEquals(0.0, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with a specific force
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        k = new BodyKinematics(fx, fy, fz);

        // check default values
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        assertEquals(fx, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(fy, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(fz, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(fx, triad1.getValueX(), 0.0);
        assertEquals(fy, triad1.getValueY(), 0.0);
        assertEquals(fz, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(0.0, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(0.0, triad2.getValueX(), 0.0);
        assertEquals(0.0, triad2.getValueY(), 0.0);
        assertEquals(0.0, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        final var normF = Math.sqrt(fx * fx + fy * fy + fz * fz);
        assertEquals(normF, k.getSpecificForceNorm(), 0.0);
        assertEquals(normF, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(0.0, k.getAngularRateNorm(), 0.0);
        assertEquals(0.0, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with specific force and angular rate
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        // check default values
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);

        assertEquals(fx, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(fy, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(fz, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(fx, triad1.getValueX(), 0.0);
        assertEquals(fy, triad1.getValueY(), 0.0);
        assertEquals(fz, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(angularRateX, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(angularRateY, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(angularRateZ, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(angularRateX, triad2.getValueX(), 0.0);
        assertEquals(angularRateY, triad2.getValueY(), 0.0);
        assertEquals(angularRateZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        final var angularRateNorm = Math.sqrt(angularRateX * angularRateX
                + angularRateY * angularRateY + angularRateZ * angularRateZ);
        assertEquals(normF, k.getSpecificForceNorm(), 0.0);
        assertEquals(normF, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(angularRateNorm, k.getAngularRateNorm(), 0.0);
        assertEquals(angularRateNorm, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with specific forces accelerations
        final var specificForceX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var specificForceY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var specificForceZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k = new BodyKinematics(specificForceX, specificForceY, specificForceZ);

        // check default values
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        assertEquals(fx, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(fy, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(fz, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(fx, triad1.getValueX(), 0.0);
        assertEquals(fy, triad1.getValueY(), 0.0);
        assertEquals(fz, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(0.0, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(0.0, triad2.getValueX(), 0.0);
        assertEquals(0.0, triad2.getValueY(), 0.0);
        assertEquals(0.0, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        assertEquals(normF, k.getSpecificForceNorm(), 0.0);
        assertEquals(normF, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(0.0, k.getAngularRateNorm(), 0.0);
        assertEquals(0.0, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with angular speeds
        final var angularSpeedX = new AngularSpeed(angularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedY = new AngularSpeed(angularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedZ = new AngularSpeed(angularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        k = new BodyKinematics(angularSpeedX, angularSpeedY, angularSpeedZ);

        // check default values
        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);

        assertEquals(0.0, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(angularRateX, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(angularRateY, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(angularRateZ, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(angularRateX, triad2.getValueX(), 0.0);
        assertEquals(angularRateY, triad2.getValueY(), 0.0);
        assertEquals(angularRateZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        assertEquals(0.0, k.getSpecificForceNorm(), 0.0);
        assertEquals(0.0, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(angularRateNorm, k.getAngularRateNorm(), 0.0);
        assertEquals(angularRateNorm, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with specific forces accelerations and angular speeds
        k = new BodyKinematics(specificForceX, specificForceY, specificForceZ, angularSpeedX, angularSpeedY, 
                angularSpeedZ);

        // check default values
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);

        assertEquals(fx, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(fy, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(fz, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(fx, triad1.getValueX(), 0.0);
        assertEquals(fy, triad1.getValueY(), 0.0);
        assertEquals(fz, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(angularRateX, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(angularRateY, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(angularRateZ, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(angularRateX, triad2.getValueX(), 0.0);
        assertEquals(angularRateY, triad2.getValueY(), 0.0);
        assertEquals(angularRateZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        assertEquals(normF, k.getSpecificForceNorm(), 0.0);
        assertEquals(normF, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(angularRateNorm, k.getAngularRateNorm(), 0.0);
        assertEquals(angularRateNorm, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test constructor with triads
        k = new BodyKinematics(triad1, triad2);

        // check default values
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);

        assertEquals(fx, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(fy, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(fz, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        triad1 = k.getSpecificForceTriad();
        assertEquals(fx, triad1.getValueX(), 0.0);
        assertEquals(fy, triad1.getValueY(), 0.0);
        assertEquals(fz, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        assertEquals(angularRateX, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(angularRateY, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(angularRateZ, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        triad2 = k.getAngularRateTriad();
        assertEquals(angularRateX, triad2.getValueX(), 0.0);
        assertEquals(angularRateY, triad2.getValueY(), 0.0);
        assertEquals(angularRateZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());

        assertEquals(normF, k.getSpecificForceNorm(), 0.0);
        assertEquals(normF, k.getSpecificForceNormAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, k.getSpecificForceNormAsAcceleration().getUnit());
        assertEquals(angularRateNorm, k.getAngularRateNorm(), 0.0);
        assertEquals(angularRateNorm, k.getAngularSpeedNorm().getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, k.getAngularSpeedNorm().getUnit());

        // test copy constructor
        final var k2 = new BodyKinematics(k);

        // check default values
        assertEquals(k.getFx(), k2.getFx(), 0.0);
        assertEquals(k.getFy(), k2.getFy(), 0.0);
        assertEquals(k.getFz(), k2.getFz(), 0.0);
        assertEquals(k.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    void testGetSetFx() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getFx(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        k.setFx(fx);

        // check
        assertEquals(fx, k.getFx(), 0.0);
    }

    @Test
    void testGetSetFy() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getFy(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        k.setFy(fy);

        // check
        assertEquals(fy, k.getFy(), 0.0);
    }

    @Test
    void testGetSetFz() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getFz(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        k.setFz(fz);

        // check
        assertEquals(fz, k.getFz(), 0.0);
    }

    @Test
    void testSetSpecificForceCoordinates() {
        final var k = new BodyKinematics();

        // check default values
        assertEquals(0.0, k.getFx(), 0.0);
        assertEquals(0.0, k.getFy(), 0.0);
        assertEquals(0.0, k.getFz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        k.setSpecificForceCoordinates(fx, fy, fz);

        // check
        assertEquals(fx, k.getFx(), 0.0);
        assertEquals(fy, k.getFy(), 0.0);
        assertEquals(fz, k.getFz(), 0.0);
    }

    @Test
    void testGetSetSpecificForceX() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getSpecificForceX().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var specificForceX1 = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceX(specificForceX1);

        // check
        final var specificForceX2 = k.getSpecificForceX();
        final var specificForceX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceX(specificForceX3);

        assertEquals(specificForceX1, specificForceX2);
        assertEquals(specificForceX1, specificForceX3);
    }

    @Test
    void testGetSetSpecificForceY() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getSpecificForceY().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var specificForceY1 = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceY(specificForceY1);

        // check
        final var specificForceY2 = k.getSpecificForceY();
        final var specificForceY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceY(specificForceY3);

        assertEquals(specificForceY1, specificForceY2);
        assertEquals(specificForceY1, specificForceY3);
    }

    @Test
    void testGetSetSpecificForceZ() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var specificForceZ1 = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceZ(specificForceZ1);

        // check
        final var specificForceZ2 = k.getSpecificForceZ();
        final var specificForceZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceZ(specificForceZ3);

        assertEquals(specificForceZ1, specificForceZ2);
        assertEquals(specificForceZ1, specificForceZ3);
    }

    @Test
    void testSetSpecificForceCoordinatesAsAcceleration() {
        final var k = new BodyKinematics();

        // check default values
        assertEquals(0.0, k.getSpecificForceX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getSpecificForceZ().getValue().doubleValue(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var specificForceX1 = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var specificForceY1 = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var specificForceZ1 = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        k.setSpecificForceCoordinates(specificForceX1, specificForceY1, specificForceZ1);

        // check
        final var specificForceX2 = k.getSpecificForceX();
        final var specificForceY2 = k.getSpecificForceY();
        final var specificForceZ2 = k.getSpecificForceZ();
        assertEquals(specificForceX1, specificForceX2);
        assertEquals(specificForceY1, specificForceY2);
        assertEquals(specificForceZ1, specificForceZ2);
    }

    @Test
    void testGetSetSpecificForceTriad() {
        final var k = new BodyKinematics();

        // check default value
        final var triad1 = k.getSpecificForceTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var triad2 = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, fx, fy, fz);
        k.setSpecificForceTriad(triad2);

        // check
        final var triad3 = k.getSpecificForceTriad();
        final var triad4 = new AccelerationTriad();
        k.getSpecificForceTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAngularRateX() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularRateX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateX(angularRateX);

        // check
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
    }

    @Test
    void testGetSetAngularRateY() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularRateY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateY(angularRateY);

        // check
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
    }

    @Test
    void testGetSetAngularRateZ() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateZ(angularRateZ);

        // check
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);
    }

    @Test
    void testSetAngularRateCoordinates() {
        final var k = new BodyKinematics();

        // check default values
        assertEquals(0.0, k.getAngularRateX(), 0.0);
        assertEquals(0.0, k.getAngularRateY(), 0.0);
        assertEquals(0.0, k.getAngularRateZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        k.setAngularRateCoordinates(angularRateX, angularRateY, angularRateZ);

        // check
        assertEquals(angularRateX, k.getAngularRateX(), 0.0);
        assertEquals(angularRateY, k.getAngularRateY(), 0.0);
        assertEquals(angularRateZ, k.getAngularRateZ(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedTriad() {
        final var k = new BodyKinematics();

        // check default value
        final var triad1 = k.getAngularRateTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, 
                angularRateX, angularRateY, angularRateZ);
        k.setAngularRateTriad(triad2);

        // check
        final var triad3 = k.getAngularRateTriad();
        final var triad4 = new AngularSpeedTriad();
        k.getAngularRateTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAngularSpeedX() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularSpeedX().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularSpeedX1 = new AngularSpeed(angularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedX(angularSpeedX1);

        // check
        final var angularSpeedX2 = k.getAngularSpeedX();
        final var angularSpeedX3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedX(angularSpeedX3);

        assertEquals(angularSpeedX1, angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX3);
    }

    @Test
    void testGetSetAngularSpeedY() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularSpeedY().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularSpeedY1 = new AngularSpeed(angularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedY(angularSpeedY1);

        // check
        final var angularSpeedY2 = k.getAngularSpeedY();
        final var angularSpeedY3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedY(angularSpeedY3);

        assertEquals(angularSpeedY1, angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY3);
    }

    @Test
    void testGetSetAngularSpeedZ() {
        final var k = new BodyKinematics();

        // check default value
        assertEquals(0.0, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularSpeedZ1 = new AngularSpeed(angularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedZ(angularSpeedZ1);

        // check
        final var angularSpeedZ2 = k.getAngularSpeedZ();
        final var angularSpeedZ3 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedZ(angularSpeedZ3);

        assertEquals(angularSpeedZ1, angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ3);
    }

    @Test
    void testSetAngularSpeedCoordinates() {
        final var k = new BodyKinematics();

        // check default values
        assertEquals(0.0, k.getAngularSpeedX().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedY().getValue().doubleValue(), 0.0);
        assertEquals(0.0, k.getAngularSpeedZ().getValue().doubleValue(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularSpeedX1 = new AngularSpeed(angularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedY1 = new AngularSpeed(angularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var angularSpeedZ1 = new AngularSpeed(angularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        k.setAngularSpeedCoordinates(angularSpeedX1, angularSpeedY1, angularSpeedZ1);

        // check
        final var angularSpeedX2 = k.getAngularSpeedX();
        final var angularSpeedY2 = k.getAngularSpeedY();
        final var angularSpeedZ2 = k.getAngularSpeedZ();
        assertEquals(angularSpeedX1, angularSpeedX2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
    }

    @Test
    void testSpecificForceNorm() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var normF = Math.sqrt(fx * fx + fy * fy + fz * fz);
        assertEquals(normF, k.getSpecificForceNorm(), 0.0);

        final var norm1 = k.getSpecificForceNormAsAcceleration();
        final var norm2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        k.getSpecificForceNormAsAcceleration(norm2);

        assertEquals(normF, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        assertEquals(norm1, norm2);
    }

    @Test
    void testAngularRateNorm() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var normAngularRate = Math.sqrt(angularRateX * angularRateX + angularRateY * angularRateY
                + angularRateZ * angularRateZ);
        assertEquals(normAngularRate, k.getAngularRateNorm(), 0.0);

        final var norm1 = k.getAngularSpeedNorm();
        final var norm2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        k.getAngularSpeedNorm(norm2);

        assertEquals(normAngularRate, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        assertEquals(norm1, norm2);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var k2 = new BodyKinematics();
        k1.copyTo(k2);

        // check
        assertEquals(k1.getFx(), k2.getFx(), 0.0);
        assertEquals(k1.getFy(), k2.getFy(), 0.0);
        assertEquals(k1.getFz(), k2.getFz(), 0.0);
        assertEquals(k1.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k1.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k1.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var k2 = new BodyKinematics();
        k2.copyFrom(k1);

        // check
        assertEquals(k1.getFx(), k2.getFx(), 0.0);
        assertEquals(k1.getFy(), k2.getFy(), 0.0);
        assertEquals(k1.getFz(), k2.getFz(), 0.0);
        assertEquals(k1.getAngularRateX(), k2.getAngularRateX(), 0.0);
        assertEquals(k1.getAngularRateY(), k2.getAngularRateY(), 0.0);
        assertEquals(k1.getAngularRateZ(), k2.getAngularRateZ(), 0.0);
    }

    @Test
    void testAsSpecificForceArray() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var array1 = new double[BodyKinematics.COMPONENTS];
        k.asSpecificForceArray(array1);
        final var array2 = k.asSpecificForceArray();

        // check
        assertEquals(fx, array1[0], 0.0);
        assertEquals(fy, array1[1], 0.0);
        assertEquals(fz, array1[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> k.asSpecificForceArray(new double[1]));
    }

    @Test
    void testAsSpecificForceMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        k.asSpecificForceMatrix(m1);
        final var m2 = k.asSpecificForceMatrix();
        final var m3 = new Matrix(1, 1);
        k.asSpecificForceMatrix(m3);

        // check
        assertEquals(fx, m1.getElementAtIndex(0), 0.0);
        assertEquals(fy, m1.getElementAtIndex(1), 0.0);
        assertEquals(fz, m1.getElementAtIndex(2), 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    void testAsAngularRateArray() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var array1 = new double[BodyKinematics.COMPONENTS];
        k.asAngularRateArray(array1);
        final var array2 = k.asAngularRateArray();

        // check
        assertEquals(angularRateX, array1[0], 0.0);
        assertEquals(angularRateY, array1[1], 0.0);
        assertEquals(angularRateZ, array1[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> k.asAngularRateArray(new double[1]));
    }

    @Test
    void testAsAngularRateMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var m1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        k.asAngularRateMatrix(m1);
        final var m2 = k.asAngularRateMatrix();
        final var m3 = new Matrix(1, 1);
        k.asAngularRateMatrix(m3);

        // check
        assertEquals(angularRateX, m1.getElementAtIndex(0), 0.0);
        assertEquals(angularRateY, m1.getElementAtIndex(1), 0.0);
        assertEquals(angularRateZ, m1.getElementAtIndex(2), 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k2 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k3 = new BodyKinematics();

        assertEquals(k1.hashCode(), k2.hashCode());
        assertNotEquals(k1.hashCode(), k3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k2 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k3 = new BodyKinematics();

        //noinspection EqualsWithItself
        assertEquals(k1, k1);
        //noinspection EqualsWithItself
        assertTrue(k1.equals(k1));
        assertTrue(k1.equals(k2));
        assertFalse(k1.equals(k3));
        assertNotEquals(null, k1);
        assertFalse(k1.equals(null));
        assertNotEquals(new Object(), k1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k2 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        final var k3 = new BodyKinematics();

        assertTrue(k1.equals(k1, THRESHOLD));
        assertTrue(k1.equals(k2, THRESHOLD));
        assertFalse(k1.equals(k3, THRESHOLD));
        assertFalse(k1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var k2 = k1.clone();

        // check
        assertEquals(k1, k2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        final var k1 = new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final var bytes = SerializationHelper.serialize(k1);
        final var k2 = SerializationHelper.deserialize(bytes);

        assertEquals(k1, k2);
        assertNotSame(k1, k2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = BodyKinematics.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
