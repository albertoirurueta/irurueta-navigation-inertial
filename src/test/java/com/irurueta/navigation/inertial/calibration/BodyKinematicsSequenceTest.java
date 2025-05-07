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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class BodyKinematicsSequenceTest {
    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    @Test
    void testConstructor1() {
        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(0, sequence.getItemsCount());
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(0.0, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(0.0, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(0.0, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor2() {
        final var items = createItems();

        final var sequence = new BodyKinematicsSequence<>(items);

        // check default values
        final var sorted1 = new ArrayList<StandardDeviationTimedBodyKinematics>();
        assertTrue(sequence.getSortedItems(sorted1));
        final var sorted2 = sequence.getSortedItems();

        // check
        assertEquals(sorted1, sorted2);

        assertEquals(2, sorted1.size());
        assertTrue(sorted1.get(0).getTimestampSeconds() < sorted1.get(1).getTimestampSeconds());
        assertEquals(2, sequence.getItemsCount());

        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(0.0, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(0.0, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(0.0, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(0.0, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(0.0, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(0.0, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor3() {
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence = new BodyKinematicsSequence<>(beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(0, sequence.getItemsCount());
        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(beforeMeanFx, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(beforeMeanFy, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(beforeMeanFz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(afterMeanFx, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(afterMeanFy, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(afterMeanFz, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor4() {
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var beforeMeanSpecificForceX = new Acceleration(beforeMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceY = new Acceleration(beforeMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceZ = new Acceleration(beforeMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceX = new Acceleration(afterMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceY = new Acceleration(afterMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceZ = new Acceleration(afterMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var sequence = new BodyKinematicsSequence<>(
                beforeMeanSpecificForceX, beforeMeanSpecificForceY, beforeMeanSpecificForceZ,
                afterMeanSpecificForceX, afterMeanSpecificForceY, afterMeanSpecificForceZ);

        // check default values
        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());
        assertEquals(0, sequence.getItemsCount());
        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(beforeMeanFx, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(beforeMeanFy, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(beforeMeanFz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(afterMeanFx, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(afterMeanFy, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(afterMeanFz, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor5() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        // check default values
        final var sorted1 = new ArrayList<StandardDeviationTimedBodyKinematics>();
        assertTrue(sequence.getSortedItems(sorted1));
        final var sorted2 = sequence.getSortedItems();

        // check
        assertEquals(sorted1, sorted2);

        assertEquals(2, sorted1.size());
        assertTrue(sorted1.get(0).getTimestampSeconds() < sorted1.get(1).getTimestampSeconds());
        assertEquals(2, sequence.getItemsCount());

        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(beforeMeanFx, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(beforeMeanFy, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(beforeMeanFz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(afterMeanFx, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(afterMeanFy, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(afterMeanFz, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor6() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var beforeMeanSpecificForceX = new Acceleration(beforeMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceY = new Acceleration(beforeMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceZ = new Acceleration(beforeMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceX = new Acceleration(afterMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceY = new Acceleration(afterMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceZ = new Acceleration(afterMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var sequence = new BodyKinematicsSequence<>(items, 
                beforeMeanSpecificForceX, beforeMeanSpecificForceY, beforeMeanSpecificForceZ,
                afterMeanSpecificForceX, afterMeanSpecificForceY, afterMeanSpecificForceZ);

        // check default values
        final var sorted1 = new ArrayList<StandardDeviationTimedBodyKinematics>();
        assertTrue(sequence.getSortedItems(sorted1));
        final var sorted2 = sequence.getSortedItems();

        // check
        assertEquals(sorted1, sorted2);

        assertEquals(2, sorted1.size());
        assertTrue(sorted1.get(0).getTimestampSeconds() < sorted1.get(1).getTimestampSeconds());
        assertEquals(2, sequence.getItemsCount());

        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);

        final var accelerationX1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(accelerationX1);
        final var accelerationX2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(beforeMeanFx, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        assertEquals(accelerationX1, accelerationX2);

        final var accelerationY1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(accelerationY1);
        final var accelerationY2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(beforeMeanFy, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        assertEquals(accelerationY1, accelerationY2);

        final var accelerationZ1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(accelerationZ1);
        final var accelerationZ2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(beforeMeanFz, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        assertEquals(accelerationZ1, accelerationZ2);

        final var accelerationX3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(accelerationX3);
        final var accelerationX4 = sequence.getAfterMeanSpecificForceX();

        assertEquals(afterMeanFx, accelerationX3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX3.getUnit());
        assertEquals(accelerationX3, accelerationX4);

        final var accelerationY3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(accelerationY3);
        final var accelerationY4 = sequence.getAfterMeanSpecificForceY();

        assertEquals(afterMeanFy, accelerationY3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY3.getUnit());
        assertEquals(accelerationY3, accelerationY4);

        final var accelerationZ3 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(accelerationZ3);
        final var accelerationZ4 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(afterMeanFz, accelerationZ3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ3.getUnit());
        assertEquals(accelerationZ3, accelerationZ4);
    }

    @Test
    void testConstructor7() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz,
                afterMeanFx, afterMeanFy, afterMeanFz);

        final var sequence2 = new BodyKinematicsSequence<>(sequence1);

        // check default values
        assertEquals(sequence1.getSortedItems(), sequence2.getSortedItems());
        assertEquals(beforeMeanFx, sequence2.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence2.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence2.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence2.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence2.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence2.getAfterMeanFz(), 0.0);

        assertEquals(sequence1, sequence2);
    }

    @Test
    void testSetItems() {
        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());

        // set items
        final var items = createItems();
        sequence.setItems(items);

        // check
        final var sorted1 = sequence.getSortedItems();
        final var sorted2 = new ArrayList<StandardDeviationTimedBodyKinematics>();
        assertTrue(sequence.getSortedItems(sorted2));

        assertEquals(2, sorted1.size());
        assertTrue(sorted1.get(0).getTimestampSeconds() < sorted1.get(1).getTimestampSeconds());
        assertEquals(sorted1, sorted2);
        assertEquals(2, sequence.getItemsCount());
    }

    @Test
    void testSetItems2() {
        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        assertFalse(sequence.getSortedItems(null));
        assertNull(sequence.getSortedItems());

        // set items
        final var items = new ArrayList<>(createItems());
        sequence.setItems(items);

        // check
        final var sorted1 = sequence.getSortedItems();
        final var sorted2 = new ArrayList<StandardDeviationTimedBodyKinematics>();
        assertTrue(sequence.getSortedItems(sorted2));

        assertEquals(2, sorted1.size());
        assertTrue(sorted1.get(0).getTimestampSeconds() < sorted1.get(1).getTimestampSeconds());
        assertEquals(sorted1, sorted2);
        assertEquals(2, sequence.getItemsCount());
    }

    @Test
    void testGetSetBeforeMeanFx() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFx(beforeMeanFx);

        // check
        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
    }

    @Test
    void testGetSetBeforeMeanFy() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFy(beforeMeanFy);

        // check
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
    }

    @Test
    void testGetSetBeforeMeanFz() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanFz(beforeMeanFz);

        // check
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
    }

    @Test
    void testSetBeforeMeanSpecificForceCoordinates1() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

        // check
        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
    }

    @Test
    void testGetSetBeforeMeanSpecificForceX() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fx1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx1);
        final var fx2 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(0.0, fx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fx1.getUnit());
        assertEquals(fx1, fx2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fx3 = new Acceleration(beforeMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceX(fx3);

        // check
        final var fx4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx4);
        final var fx5 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(fx3, fx4);
        assertEquals(fx3, fx5);
    }

    @Test
    void testGetSetBeforeMeanSpecificForceY() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fy1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(fy1);
        final var fy2 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(0.0, fy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fy1.getUnit());
        assertEquals(fy1, fy2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var meanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy3 = new Acceleration(meanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceY(fy3);

        // check
        final var fy4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceY(fy4);
        final var fy5 = sequence.getBeforeMeanSpecificForceY();

        assertEquals(fy3, fy4);
        assertEquals(fy3, fy5);
    }

    @Test
    void testGetSetBeforeMeanSpecificForceZ() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fz1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(fz1);
        final var fz2 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(0.0, fz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fz1.getUnit());
        assertEquals(fz1, fz2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var meanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz3 = new Acceleration(meanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceZ(fz3);

        // check
        final var fz4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceZ(fz4);
        final var fz5 = sequence.getBeforeMeanSpecificForceZ();

        assertEquals(fz3, fz4);
        assertEquals(fz3, fz5);
    }

    @Test
    void testSetBeforeMeanSpecificForceCoordinates2() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var beforeMeanSpecificForceX = new Acceleration(beforeMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceY = new Acceleration(beforeMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var beforeMeanSpecificForceZ = new Acceleration(beforeMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        sequence.setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX, beforeMeanSpecificForceY, beforeMeanSpecificForceZ);

        // check
        assertEquals(beforeMeanFx, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence.getBeforeMeanFz(), 0.0);
    }

    @Test
    void testGetSetAfterMeanFx() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFx(afterMeanFx);

        // check
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
    }

    @Test
    void testGetSetAfterMeanFy() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFy(afterMeanFy);

        // check
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
    }

    @Test
    void testGetSetAfterMeanFz() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanFz(afterMeanFz);

        // check
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);
    }

    @Test
    void testSetAfterMeanSpecificForceCoordinates1() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

        // check
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);
    }

    @Test
    void testGetSetAfterMeanSpecificForceX() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fx1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceX(fx1);
        final var fx2 = sequence.getAfterMeanSpecificForceX();

        assertEquals(0.0, fx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fx1.getUnit());
        assertEquals(fx1, fx2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fx3 = new Acceleration(afterMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setBeforeMeanSpecificForceX(fx3);

        // check
        final var fx4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getBeforeMeanSpecificForceX(fx4);
        final var fx5 = sequence.getBeforeMeanSpecificForceX();

        assertEquals(fx3, fx4);
        assertEquals(fx3, fx5);
    }

    @Test
    void testGetSetAfterMeanSpecificForceY() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fy1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(fy1);
        final var fy2 = sequence.getAfterMeanSpecificForceY();

        assertEquals(0.0, fy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fy1.getUnit());
        assertEquals(fy1, fy2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var meanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy3 = new Acceleration(meanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setAfterMeanSpecificForceY(fy3);

        // check
        final var fy4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceY(fy4);
        final var fy5 = sequence.getAfterMeanSpecificForceY();

        assertEquals(fy3, fy4);
        assertEquals(fy3, fy5);
    }

    @Test
    void testGetSetAfterMeanSpecificForceZ() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        final var fz1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(fz1);
        final var fz2 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(0.0, fz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, fz1.getUnit());
        assertEquals(fz1, fz2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var meanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz3 = new Acceleration(meanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        sequence.setAfterMeanSpecificForceZ(fz3);

        // check
        final var fz4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        sequence.getAfterMeanSpecificForceZ(fz4);
        final var fz5 = sequence.getAfterMeanSpecificForceZ();

        assertEquals(fz3, fz4);
        assertEquals(fz3, fz5);
    }

    @Test
    void testSetAfterMeanSpecificForceCoordinates2() {
        final var sequence = new BodyKinematicsSequence<>();

        // check default value
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.0);
        assertEquals(0.0, sequence.getBeforeMeanFz(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFx(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFy(), 0.0);
        assertEquals(0.0, sequence.getAfterMeanFz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var afterMeanSpecificForceX = new Acceleration(afterMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceY = new Acceleration(afterMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var afterMeanSpecificForceZ = new Acceleration(afterMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        sequence.setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX, afterMeanSpecificForceY, afterMeanSpecificForceZ);

        // check
        assertEquals(afterMeanFx, sequence.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence.getAfterMeanFz(), 0.0);
    }

    @Test
    void testCopyFrom1() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        final var sequence2 = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertEquals(sequence2.getSortedItems().size(), items.size());
        assertEquals(beforeMeanFx, sequence2.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence2.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence2.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence2.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence2.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence2.getAfterMeanFz(), 0.0);
    }

    @Test
    void testCopyFrom2() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz,
                afterMeanFx, afterMeanFy, afterMeanFz);

        assertNotNull(sequence1.getSortedItems());

        final var sequence2 = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertEquals(items.size(), sequence2.getSortedItems().size());
        assertEquals(beforeMeanFx, sequence2.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence2.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence2.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence2.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence2.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence2.getAfterMeanFz(), 0.0);
    }

    @Test
    void testCopyFrom3() {
        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        final var sequence2 = new BodyKinematicsSequence<>();

        sequence2.copyFrom(sequence1);

        assertEquals(sequence1, sequence2);

        assertNull(sequence2.getSortedItems());
        assertEquals(beforeMeanFx, sequence2.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence2.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence2.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence2.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence2.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence2.getAfterMeanFz(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        final var sequence2 = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();

        sequence1.copyTo(sequence2);

        assertEquals(sequence1, sequence2);

        assertEquals(items.size(), sequence2.getSortedItems().size());
        assertEquals(beforeMeanFx, sequence2.getBeforeMeanFx(), 0.0);
        assertEquals(beforeMeanFy, sequence2.getBeforeMeanFy(), 0.0);
        assertEquals(beforeMeanFz, sequence2.getBeforeMeanFz(), 0.0);
        assertEquals(afterMeanFx, sequence2.getAfterMeanFx(), 0.0);
        assertEquals(afterMeanFy, sequence2.getAfterMeanFy(), 0.0);
        assertEquals(afterMeanFz, sequence2.getAfterMeanFz(), 0.0);
    }

    @Test
    void testEqualsAndHashCode() {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);
        final var sequence2 = new BodyKinematicsSequence<>(sequence1);
        final var sequence3 = new BodyKinematicsSequence<>();

        assertEquals(sequence1, sequence2);
        assertNotEquals(sequence1, sequence3);

        assertEquals(sequence1.hashCode(), sequence2.hashCode());
        assertNotEquals(sequence1.hashCode(), sequence3.hashCode());
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);
        final var sequence2 = sequence1.clone();

        assertEquals(sequence1, sequence2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var items = createItems();

        final var randomizer = new UniformRandomizer();
        final var beforeMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var beforeMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var afterMeanFz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var sequence1 = new BodyKinematicsSequence<>(items, beforeMeanFx, beforeMeanFy, beforeMeanFz, 
                afterMeanFx, afterMeanFy, afterMeanFz);

        final var bytes = SerializationHelper.serialize(sequence1);
        final var sequence2 = SerializationHelper.deserialize(bytes);

        assertEquals(sequence1, sequence2);
        assertNotSame(sequence1, sequence2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = BodyKinematicsSequence.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    private static List<StandardDeviationTimedBodyKinematics> createItems() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        return Arrays.asList(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
    }

    private static BodyKinematics createBodyKinematics() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }
}
