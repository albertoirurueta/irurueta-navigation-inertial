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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class INSTightlyCoupledKalmanInitializerTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int NUM_PARAMS = INSTightlyCoupledKalmanInitializer.NUM_PARAMS;

    @Test
    void testInitialize() throws WrongSizeException {
        final var config = generateConfig();

        final var expected = Matrix.diagonal(new double[]{
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialAttitudeUncertainty() * config.getInitialAttitudeUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialVelocityUncertainty() * config.getInitialVelocityUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialPositionUncertainty() * config.getInitialPositionUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialAccelerationBiasUncertainty() * config.getInitialAccelerationBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialGyroscopeBiasUncertainty() * config.getInitialGyroscopeBiasUncertainty(),
                config.getInitialClockOffsetUncertainty() * config.getInitialClockOffsetUncertainty(),
                config.getInitialClockDriftUncertainty() * config.getInitialClockDriftUncertainty()
        });

        final var result1 = new Matrix(NUM_PARAMS, NUM_PARAMS);
        INSTightlyCoupledKalmanInitializer.initialize(config, result1);
        final var result2 = new Matrix(1, 1);
        INSTightlyCoupledKalmanInitializer.initialize(config, result2);
        final var result3 = INSTightlyCoupledKalmanInitializer.initialize(config);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
        assertEquals(expected, result3);
    }

    private static INSTightlyCoupledKalmanInitializerConfig generateConfig() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        return new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);
    }
}
