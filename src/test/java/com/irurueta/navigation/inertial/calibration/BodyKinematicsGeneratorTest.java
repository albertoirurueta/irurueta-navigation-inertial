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

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
class BodyKinematicsGeneratorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ACCEL_QUANT_RESIDUAL = -5e-3;
    private static final double MAX_ACCEL_QUANT_RESIDUAL = 5e-3;

    private static final double MIN_GYRO_QUANT_RESIDUAL = -5e-5;
    private static final double MAX_GYRO_QUANT_RESIDUAL = 5e-5;

    private static final int NUM = 10;

    private static final int SAMPLES = 100000;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final double ACCEL_QUANT_LEVEL = 1e-2;

    private static final double GYRO_QUANT_LEVEL = 2e-4;

    @Test
    void testGenerateSingleTimeIntervalQuantizedAndWithNoise() throws WrongSizeException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, ACCEL_QUANT_LEVEL,
                GYRO_QUANT_LEVEL);
        
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var oldQuantizationResiduals = new double[6];
        for (var i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (var i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final var quantizationResiduals1 = new double[6];
        final var result1 = new BodyKinematics();
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, 
                oldQuantizationResiduals, result1, quantizationResiduals1);
        final var quantizationResiduals2 = new double[6];
        final var result2 = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                oldQuantizationResiduals, quantizationResiduals2);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result3 = new BodyKinematics();
        final var quantizationResiduals3 = new double[6];
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, oldQuantizationResiduals,
                result3, quantizationResiduals3);

        final var quantizationResiduals4 = new double[6];
        final var result4 = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random,
                oldQuantizationResiduals, quantizationResiduals4);

        final var expectedQuantizationResiduals = new double[6];
        final var expected = generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                oldQuantizationResiduals, expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1, ABSOLUTE_ERROR);

        assertTrue(expected.equals(result2, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals2, ABSOLUTE_ERROR);

        assertTrue(expected.equals(result3, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals3, ABSOLUTE_ERROR);

        assertTrue(expected.equals(result4, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals4, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, new double[1], result1, quantizationResiduals1));
        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, oldQuantizationResiduals, result1, new double[1]));

        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, new double[1], quantizationResiduals2));
        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random, oldQuantizationResiduals, new double[1]));

        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors, random, new double[1], result3, quantizationResiduals3));
        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors, random, oldQuantizationResiduals, result3, new double[1]));

        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors, random, new double[1], quantizationResiduals4));
        assertThrows(IllegalArgumentException.class, () -> BodyKinematicsGenerator.generate(timeInterval,
                trueKinematics, errors, random, oldQuantizationResiduals, new double[1]));
    }

    @Test
    void testGenerateSingleNoTimeIntervalQuantizedAndWithNoise() throws WrongSizeException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, ACCEL_QUANT_LEVEL,
                GYRO_QUANT_LEVEL);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var oldQuantizationResiduals = new double[6];
        for (var i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (var i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final var quantizationResiduals1 = new double[6];
        final var result1 = new BodyKinematics();
        final var random = mock(Random.class);

        BodyKinematicsGenerator.generate(0.0, trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);

        final var expectedQuantizationResiduals = new double[6];
        final var expected = generate(0.0, trueKinematics, errors, random, oldQuantizationResiduals,
                expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1, ABSOLUTE_ERROR);

        verifyNoInteractions(random);
    }

    @Test
    void testGenerateSingleTimeIntervalQuantizedAndWithoutNoise() throws WrongSizeException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, ACCEL_QUANT_LEVEL,
                GYRO_QUANT_LEVEL);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var oldQuantizationResiduals = new double[6];
        for (var i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (var i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final var quantizationResiduals1 = new double[6];
        final var result1 = new BodyKinematics();
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                oldQuantizationResiduals, result1, quantizationResiduals1);

        // When no noise is present, is equivalent to zero time intervals
        final var expectedQuantizationResiduals = new double[6];
        final var expected = generate(0.0, trueKinematics, errors, random, oldQuantizationResiduals,
                expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1, ABSOLUTE_ERROR);

        verify(random, atLeast(6)).nextGaussian();
    }

    @Test
    void testGenerateSingleTimeIntervalNotQuantizedAndWithNoise() throws WrongSizeException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel1 = 0.0;
        final var gyroQuantLevel1 = 0.0;

        final var errors1 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel1,
                gyroQuantLevel1);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var oldQuantizationResiduals = new double[6];
        for (var i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (var i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final var quantizationResiduals1 = new double[6];
        final var result1 = new BodyKinematics();
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random,
                oldQuantizationResiduals, result1, quantizationResiduals1);

        final var errors2 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, ACCEL_QUANT_LEVEL,
                GYRO_QUANT_LEVEL);

        // when no old quantization residuals are provided, quantization gets disabled
        final var quantizationResiduals2 = new double[6];
        final var result2 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors2, random,
                null, result2, quantizationResiduals2);

        final var result3 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random,
                oldQuantizationResiduals, result3, null);

        final var result4 = new BodyKinematics();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random, result4);

        final var result5 = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result6 = new BodyKinematics();
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1, random, result6);

        final var result7 = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1, random);

        final var expectedQuantizationResiduals = new double[6];
        final var expected = generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random, oldQuantizationResiduals,
                expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1, ABSOLUTE_ERROR);

        assertTrue(expected.equals(result2, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals2, ABSOLUTE_ERROR);

        assertTrue(expected.equals(result3, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result4, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result5, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result6, ABSOLUTE_ERROR));
        assertTrue(expected.equals(result7, ABSOLUTE_ERROR));

        assertArrayEquals(new double[6], quantizationResiduals1, 0.0);
        assertArrayEquals(new double[6], quantizationResiduals2, 0.0);
    }

    @Test
    void testGenerateSingleNoTimeIntervalNoQuantizationAndNoNoise() throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var oldQuantizationResiduals = new double[6];
        for (var i = 0; i < 3; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_ACCEL_QUANT_RESIDUAL, MAX_ACCEL_QUANT_RESIDUAL);
        }
        for (var i = 3; i < 6; i++) {
            oldQuantizationResiduals[i] = randomizer.nextDouble(MIN_GYRO_QUANT_RESIDUAL, MAX_GYRO_QUANT_RESIDUAL);
        }

        final var quantizationResiduals1 = new double[6];
        final var result1 = new BodyKinematics();
        final var random = mock(Random.class);

        BodyKinematicsGenerator.generate(0.0, trueKinematics, errors, random, oldQuantizationResiduals,
                result1, quantizationResiduals1);

        final var expectedQuantizationResiduals = new double[6];
        final var expected = generate(0.0, trueKinematics, errors, random, oldQuantizationResiduals,
                expectedQuantizationResiduals);

        assertTrue(expected.equals(result1, ABSOLUTE_ERROR));
        assertArrayEquals(expectedQuantizationResiduals, quantizationResiduals1, ABSOLUTE_ERROR);

        verifyNoInteractions(random);

        // Because we follow the model
        // uf = ba + (I + Ma)*f + noise
        // ug = bg + (I + Mg)*g + Gg*f + noise

        // Neglecting noise terms,
        // then true values can be recovered as:
        // f = (I + Ma)^-1*(uf - ba)
        // g = (I + Mg)^-1*(ug - bg - Gg*f)

        // Hence:
        final var uf = result1.asSpecificForceMatrix();
        final var ug = result1.asAngularRateMatrix();

        final var identity = Matrix.identity(3, 3);
        final var f = Utils.inverse(identity.addAndReturnNew(ma)).multiplyAndReturnNew(uf.subtractAndReturnNew(ba));

        assertTrue(f.equals(trueKinematics.asSpecificForceMatrix(), ABSOLUTE_ERROR));

        final var g = Utils.inverse(identity.addAndReturnNew(mg)).multiplyAndReturnNew(ug.subtractAndReturnNew(bg)
                .subtractAndReturnNew(gg.multiplyAndReturnNew(f)));

        assertTrue(g.equals(trueKinematics.asAngularRateMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testGenerateCollection() throws WrongSizeException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();

        final var errors1 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, ACCEL_QUANT_LEVEL,
                GYRO_QUANT_LEVEL);
        final var errors2 = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                0.0, 0.0);

        final var randomizer = new UniformRandomizer();
        final var oldQuantizationResiduals = new double[6];
        final var quantizationResiduals = new double[6];

        final var trueKinematics = new ArrayList<BodyKinematics>();
        final var expected = new ArrayList<BodyKinematics>();
        final var random = mock(Random.class);
        when(random.nextGaussian()).thenReturn(0.5);

        for (var i = 0; i < NUM; i++) {
            final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

            trueKinematics.add(new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ));

            expected.add(generate(TIME_INTERVAL_SECONDS, trueKinematics.get(i), errors2, random,
                    oldQuantizationResiduals, quantizationResiduals));
        }

        final var result1 = new ArrayList<BodyKinematics>();
        BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random, result1);
        final var result2 = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors1, random);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result3 = new ArrayList<BodyKinematics>();
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1, random, result3);

        final var result4 = BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors1, random);

        assertEquals(expected, result1);
        assertEquals(expected, result2);
        assertEquals(expected, result3);
        assertEquals(expected, result4);
    }

    @Test
    void testEstimateNoiseRootPSDs() throws WrongSizeException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();

        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var result = new BodyKinematics();
        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        var avgOmegaX = 0.0;
        var avgOmegaY = 0.0;
        var avgOmegaZ = 0.0;
        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        var varOmegaX = 0.0;
        var varOmegaY = 0.0;
        var varOmegaZ = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < SAMPLES; i++, j++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, result);

            avgFx = avgFx * (double) i / (double) j + result.getFx() / j;
            avgFy = avgFy * (double) i / (double) j + result.getFy() / j;
            avgFz = avgFz * (double) i / (double) j + result.getFz() / j;

            avgOmegaX = avgOmegaX * (double) i / (double) j + result.getAngularRateX() / j;
            avgOmegaY = avgOmegaY * (double) i / (double) j + result.getAngularRateY() / j;
            avgOmegaZ = avgOmegaZ * (double) i / (double) j + result.getAngularRateZ() / j;

            var diff = result.getFx() - avgFx;
            var diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = result.getFy() - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = result.getFz() - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateX() - avgOmegaX;
            diff2 = diff * diff;
            varOmegaX = varOmegaX * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateY() - avgOmegaY;
            diff2 = diff * diff;
            varOmegaY = varOmegaY * (double) i / (double) j + diff2 / j;

            diff = result.getAngularRateZ() - avgOmegaZ;
            diff2 = diff * diff;
            varOmegaZ = varOmegaZ * (double) i / (double) j + diff2 / j;
        }

        final var rootPsdFx = Math.sqrt(varFx * TIME_INTERVAL_SECONDS);
        final var rootPsdFy = Math.sqrt(varFy * TIME_INTERVAL_SECONDS);
        final var rootPsdFz = Math.sqrt(varFz * TIME_INTERVAL_SECONDS);
        final var rootPsdOmegaX = Math.sqrt(varOmegaX * TIME_INTERVAL_SECONDS);
        final var rootPsdOmegaY = Math.sqrt(varOmegaY * TIME_INTERVAL_SECONDS);
        final var rootPsdOmegaZ = Math.sqrt(varOmegaZ * TIME_INTERVAL_SECONDS);

        assertEquals(rootPsdFx, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, LARGE_ABSOLUTE_ERROR);

        assertEquals(rootPsdOmegaX, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaY, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaZ, gyroNoiseRootPSD, LARGE_ABSOLUTE_ERROR);
    }

    private static BodyKinematics generate(
            final double timeInterval, final BodyKinematics trueKinematics, final IMUErrors errors, final Random random,
            final double[] oldQuantizationResiduals, final double[] quantizationResiduals) throws WrongSizeException {

        final var accelNoise = new Matrix(3, 1);
        final var gyroNoise = new Matrix(3, 1);
        if (timeInterval > 0.0) {
            for (var i = 0; i < 3; i++) {
                accelNoise.setElementAtIndex(i, random.nextGaussian() * errors.getAccelerometerNoiseRootPSD()
                        / Math.sqrt(timeInterval));
                gyroNoise.setElementAtIndex(i, random.nextGaussian() * errors.getGyroNoiseRootPSD()
                        / Math.sqrt(timeInterval));
            }
        }

        final var trueFibb = trueKinematics.asSpecificForceMatrix();
        final var trueOmegaIbb = trueKinematics.asAngularRateMatrix();

        final var identity = Matrix.identity(3, 3);

        final var ba = errors.getAccelerometerBiasesAsMatrix();
        final var ma = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();

        final var bg = errors.getGyroBiasesAsMatrix();
        final var mg = errors.getGyroScaleFactorAndCrossCouplingErrors();
        final var gg = errors.getGyroGDependentBiases();

        final var uqFibb = ba.addAndReturnNew((identity.addAndReturnNew(ma))
                .multiplyAndReturnNew(trueFibb))
                .addAndReturnNew(accelNoise);
        final var uqOmegaibb = bg.addAndReturnNew((identity.addAndReturnNew(mg))
                .multiplyAndReturnNew(trueOmegaIbb))
                .addAndReturnNew(gg.multiplyAndReturnNew(trueFibb))
                .addAndReturnNew(gyroNoise);

        final var oldRes = Matrix.newFromArray(oldQuantizationResiduals);

        final var accelQuantLevel = errors.getAccelerometerQuantizationLevel();
        final Matrix measFibb;
        if (accelQuantLevel > 0.0) {
            final var tmp = (uqFibb.addAndReturnNew(oldRes
                    .getSubmatrix(0, 0, 2, 0)))
                    .multiplyByScalarAndReturnNew(1.0 / accelQuantLevel);
            for (var i = 0; i < 3; i++) {
                tmp.setElementAtIndex(i, Math.round(tmp.getElementAtIndex(i)));
            }
            measFibb = tmp.multiplyByScalarAndReturnNew(accelQuantLevel);

            for (var i = 0; i < 3; i++) {
                quantizationResiduals[i] = uqFibb.getElementAtIndex(i) + oldQuantizationResiduals[i]
                        - measFibb.getElementAtIndex(i);
            }
        } else {
            measFibb = uqFibb;

            for (var i = 0; i < 3; i++) {
                quantizationResiduals[i] = 0.0;
            }
        }

        final var gyroQuantLevel = errors.getGyroQuantizationLevel();
        final Matrix measOmegaIbb;
        if (gyroQuantLevel > 0.0) {
            final var tmp = (uqOmegaibb.addAndReturnNew(oldRes
                    .getSubmatrix(3, 0, 5, 0)))
                    .multiplyByScalarAndReturnNew(1.0 / gyroQuantLevel);
            for (var i = 0; i < 3; i++) {
                tmp.setElementAtIndex(i, Math.round(tmp.getElementAtIndex(i)));
            }
            measOmegaIbb = tmp.multiplyByScalarAndReturnNew(gyroQuantLevel);


            for (int i = 0, j = 3; i < 3; i++, j++) {
                quantizationResiduals[j] = uqOmegaibb.getElementAtIndex(i) + oldQuantizationResiduals[j]
                        - measOmegaIbb.getElementAtIndex(i);
            }
        } else {
            measOmegaIbb = uqOmegaibb;


            for (var i = 3; i < 6; i++) {
                quantizationResiduals[i] = 0.0;
            }
        }

        return new BodyKinematics(measFibb.getElementAtIndex(0), measFibb.getElementAtIndex(1),
                measFibb.getElementAtIndex(2), measOmegaIbb.getElementAtIndex(0), measOmegaIbb.getElementAtIndex(1),
                measOmegaIbb.getElementAtIndex(2));
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMa() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
