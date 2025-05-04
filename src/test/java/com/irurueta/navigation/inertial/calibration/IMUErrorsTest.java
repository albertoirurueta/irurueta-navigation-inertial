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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class IMUErrorsTest {

    private static final int COMPONENTS = 3;

    private static final double MIN_VALUE = -1.0;
    private static final double MAX_VALUE = 1.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstructors() throws WrongSizeException {
        // test constructor 1
        var errors = new IMUErrors();

        // check default values
        final var zeros = new double[COMPONENTS];
        assertArrayEquals(zeros, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(zeros, errors.getGyroBiases(), 0.0);
        assertEquals(Matrix.identity(COMPONENTS, COMPONENTS),
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(Matrix.identity(COMPONENTS, COMPONENTS), errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(new Matrix(COMPONENTS, COMPONENTS), errors.getGyroGDependentBiases());
        assertEquals(0.0, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, errors.getGyroQuantizationLevel(), 0.0);

        // test constructor 2
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors = new IMUErrors(accelerometerBiases, gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(new Matrix(COMPONENTS, COMPONENTS), errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        var wrongArray = new double[1];
        final var wrongMatrix1 = new Matrix(1, 3);
        final var wrongMatrix2 = new Matrix(4, 1);

        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongArray, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, wrongArray,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases, wrongMatrix1,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases, wrongMatrix2,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));

        // test constructor 3
        final var accelerometerBiasesMatrix = new Matrix(3, 1);
        accelerometerBiasesMatrix.fromArray(accelerometerBiases);

        final var gyroBiasesMatrix = new Matrix(3, 1);
        gyroBiasesMatrix.fromArray(gyroBiases);

        errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix, 
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors, 
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(new Matrix(COMPONENTS, COMPONENTS), errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongMatrix1, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongMatrix2, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, wrongMatrix1,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, wrongMatrix2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));

        // test constructor 4
        final var accelerometerBiases2 = new Acceleration[COMPONENTS];
        accelerometerBiases2[0] = new Acceleration(accelerometerBiases[0], AccelerationUnit.METERS_PER_SQUARED_SECOND);
        accelerometerBiases2[1] = new Acceleration(accelerometerBiases[1], AccelerationUnit.METERS_PER_SQUARED_SECOND);
        accelerometerBiases2[2] = new Acceleration(accelerometerBiases[2], AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var gyroBiases2 = new AngularSpeed[COMPONENTS];
        gyroBiases2[0] = new AngularSpeed(gyroBiases[0], AngularSpeedUnit.RADIANS_PER_SECOND);
        gyroBiases2[1] = new AngularSpeed(gyroBiases[1], AngularSpeedUnit.RADIANS_PER_SECOND);
        gyroBiases2[2] = new AngularSpeed(gyroBiases[2], AngularSpeedUnit.RADIANS_PER_SECOND);

        errors = new IMUErrors(accelerometerBiases2, gyroBiases2, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(new Matrix(COMPONENTS, COMPONENTS), errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        final var wrongAccelerations = new Acceleration[1];
        final var wrongAngularSpeed = new AngularSpeed[1];

        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongAccelerations, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, wrongAngularSpeed,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors, accelerometerNoiseRootPSD, gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD));

        // test constructor 5
        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors = new IMUErrors(accelerometerBiases, gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongArray, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, wrongArray,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases, wrongMatrix1,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases, wrongMatrix2,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases,
                gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                wrongMatrix2, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));

        // test constructor 6
        errors = new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix, 
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongMatrix1, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongMatrix2, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, wrongMatrix1,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, wrongMatrix2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiasesMatrix, gyroBiasesMatrix,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel));

        // test constructor 7
        final var accelerometerQuantizationLevel2 = new Acceleration(accelerometerQuantizationLevel, 
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var gyroQuantizationLevel2 = new AngularSpeed(gyroQuantizationLevel, AngularSpeedUnit.RADIANS_PER_SECOND);
        errors = new IMUErrors(accelerometerBiases2, gyroBiases2, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2);

        // check default values
        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors, errors.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors.getGyroQuantizationLevel(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(wrongAccelerations, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2,
                gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, wrongAngularSpeed,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2,
                gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                wrongMatrix1, gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                wrongMatrix2, gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix1, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, wrongMatrix2, gyroGDependenciesBiases,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors, wrongMatrix1,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));
        assertThrows(IllegalArgumentException.class, () -> new IMUErrors(accelerometerBiases2, gyroBiases2,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors, wrongMatrix2,
                accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel2, gyroQuantizationLevel2));

        // test constructor 8
        errors = new IMUErrors(accelerometerBiases, gyroBiases, accelerometerScaleFactorAndCrossCouplingErrors,
                gyroScaleFactorAndCrossCouplingErrors, gyroGDependenciesBiases, accelerometerNoiseRootPSD,
                gyroNoiseRootPSD, accelerometerQuantizationLevel, gyroQuantizationLevel);

        final var errors2 = new IMUErrors(errors);

        // check default values
        assertArrayEquals(accelerometerBiases, errors2.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors2.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors2.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors2.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors2.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors2.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors2.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors2.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors2.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiases() {
        final var errors = new IMUErrors();

        // check default value
        final var zeros = new double[COMPONENTS];
        assertArrayEquals(zeros, errors.getAccelerometerBiases(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerBiases(accelerometerBiases);

        assertArrayEquals(accelerometerBiases, errors.getAccelerometerBiases(), 0.0);
        final var result = new double[COMPONENTS];
        errors.getAccelerometerBiases(result);
        assertArrayEquals(result, accelerometerBiases, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerBiases(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> errors.getAccelerometerBiases(new double[1]));
    }

    @Test
    void testGetSetAccelerometerBiasesAsMatrix() throws WrongSizeException {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(new Matrix(COMPONENTS, 1), errors.getAccelerometerBiasesAsMatrix());

        // set new value
        final var randomizer = new UniformRandomizer();

        final var array = new double[COMPONENTS];
        randomizer.fill(array, MIN_VALUE, MAX_VALUE);

        final var accelerometerBiases1 = Matrix.newFromArray(array);

        errors.setAccelerometerBiases(accelerometerBiases1);

        // check
        final var accelerometerBiases2 = new Matrix(COMPONENTS, 1);
        errors.getAccelerometerBiasesAsMatrix(accelerometerBiases2);
        final var accelerometerBiases3 = errors.getAccelerometerBiasesAsMatrix();

        assertEquals(accelerometerBiases1, accelerometerBiases2);
        assertEquals(accelerometerBiases1, accelerometerBiases3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.getAccelerometerBiasesAsMatrix(m1));
        final var m2 = new Matrix(COMPONENTS, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerBiases(m2));
        final var m3 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerBiases(m3));
    }

    @Test
    void testGetSetAccelerometerBiasesAsAcceleration() {
        final var errors = new IMUErrors();

        // check default value
        final var accelerations1 = errors.getAccelerometerBiasesAsAcceleration();
        for (final var a : accelerations1) {
            assertEquals(new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND), a);
        }

        // set new values
        final var randomizer = new UniformRandomizer();
        final var accelerations2 = new Acceleration[COMPONENTS];
        for (var i = 0; i < COMPONENTS; i++) {
            accelerations2[i] = new Acceleration(randomizer.nextDouble(MIN_VALUE, MAX_VALUE),
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
        }

        errors.setAccelerometerBiases(accelerations2);

        // check
        final var accelerations3 = errors.getAccelerometerBiasesAsAcceleration();
        final var accelerations4 = new Acceleration[COMPONENTS];
        accelerations4[0] = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        errors.getAccelerometerBiasesAsAcceleration(accelerations4);

        assertArrayEquals(accelerations2, accelerations3);
        assertArrayEquals(accelerations2, accelerations4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> errors.getAccelerometerBiasesAsAcceleration(new Acceleration[1]));
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerBiases(new Acceleration[1]));
    }

    @Test
    void testGetSetGyroBiases() {
        final var errors = new IMUErrors();

        // check default value
        final var zeros = new double[COMPONENTS];
        assertArrayEquals(zeros, errors.getGyroBiases(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        errors.setGyroBiases(gyroBiases);

        assertArrayEquals(gyroBiases, errors.getGyroBiases(), 0.0);
        final var result = new double[COMPONENTS];
        errors.getGyroBiases(result);
        assertArrayEquals(result, gyroBiases, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroBiases(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> errors.getGyroBiases(new double[1]));
    }

    @Test
    void testGetSetGyroBiasesAsMatrix() throws WrongSizeException {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(new Matrix(COMPONENTS, 1), errors.getGyroBiasesAsMatrix());

        // set new value
        final var randomizer = new UniformRandomizer();

        final var array = new double[COMPONENTS];
        randomizer.fill(array, MIN_VALUE, MAX_VALUE);

        final var gyroBiases1 = Matrix.newFromArray(array);

        errors.setGyroBiases(gyroBiases1);

        // check
        final var gyroBiases2 = new Matrix(COMPONENTS, 1);
        errors.getGyroBiasesAsMatrix(gyroBiases2);
        final var gyroBiases3 = errors.getGyroBiasesAsMatrix();

        assertEquals(gyroBiases1, gyroBiases2);
        assertEquals(gyroBiases1, gyroBiases3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.getGyroBiasesAsMatrix(m1));
        final var m2 = new Matrix(COMPONENTS, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroBiases(m2));
        final var m3 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroBiases(m3));
    }

    @Test
    void testGetSetGyroBiasesAsAngularSpeed() {
        final var errors = new IMUErrors();

        // check default value
        final var array1 = errors.getGyroBiasesAsAngularSpeed();
        for (final var as : array1) {
            assertEquals(new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND), as);
        }

        // set new value
        final var randomizer = new UniformRandomizer();
        final var array2 = new AngularSpeed[COMPONENTS];
        for (var i = 0; i < COMPONENTS; i++) {
            array2[i] = new AngularSpeed(randomizer.nextDouble(MIN_VALUE, MAX_VALUE),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
        }

        errors.setGyroBiases(array2);

        // check
        final var array3 = errors.getGyroBiasesAsAngularSpeed();
        final var array4 = new AngularSpeed[COMPONENTS];
        array4[0] = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        errors.getGyroBiasesAsAngularSpeed(array4);

        assertArrayEquals(array2, array3);
        assertArrayEquals(array2, array4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> errors.getGyroBiasesAsAngularSpeed(new AngularSpeed[1]));
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroBiases(new AngularSpeed[1]));
    }

    @Test
    void testGetSetAccelerometerScaleFactorAndCrossCouplingErrors() throws WrongSizeException {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(Matrix.identity(COMPONENTS, COMPONENTS),
                errors.getAccelerometerScaleFactorAndCrossCouplingErrors());

        // set new value
        final var ma1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, ma1);

        errors.setAccelerometerScaleFactorAndCrossCouplingErrors(ma1);

        // check
        final var ma2 = errors.getAccelerometerScaleFactorAndCrossCouplingErrors();
        final var ma3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getAccelerometerScaleFactorAndCrossCouplingErrors(ma3);

        assertEquals(ma1, ma2);
        assertEquals(ma1, ma3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(COMPONENTS, 1);
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerScaleFactorAndCrossCouplingErrors(
                m1));
        final var m2 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerScaleFactorAndCrossCouplingErrors(
                m2));
    }

    @Test
    void testGetSetGyroScaleFactorAndCrossCouplingErrors() throws WrongSizeException {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(Matrix.identity(COMPONENTS, COMPONENTS), errors.getGyroScaleFactorAndCrossCouplingErrors());

        // set new value
        final var mg1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, mg1);

        errors.setGyroScaleFactorAndCrossCouplingErrors(mg1);

        // check
        final var mg2 = errors.getGyroScaleFactorAndCrossCouplingErrors();
        final var mg3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getGyroScaleFactorAndCrossCouplingErrors(mg3);

        assertEquals(mg1, mg2);
        assertEquals(mg1, mg3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(COMPONENTS, 1);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroScaleFactorAndCrossCouplingErrors(m1));
        final var m2 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroScaleFactorAndCrossCouplingErrors(m2));
    }

    @Test
    void testGetSetGyroGDependentBiases() throws WrongSizeException {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(new Matrix(COMPONENTS, COMPONENTS), errors.getGyroGDependentBiases());

        // set new value
        final var mf1 = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, mf1);

        errors.setGyroGDependentBiases(mf1);

        // check
        final var mf2 = errors.getGyroGDependentBiases();
        final var mf3 = new Matrix(COMPONENTS, COMPONENTS);
        errors.getGyroGDependentBiases(mf3);

        assertEquals(mf1, mf2);
        assertEquals(mf1, mf3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(COMPONENTS, 1);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroGDependentBiases(m1));
        final var m2 = new Matrix(1, COMPONENTS);
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroGDependentBiases(m2));
    }

    @Test
    void testGetSetAccelerometerNoiseRootPSD() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerNoisePSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerNoiseRootPSD(accelerometerNoiseRootPSD);

        // check
        assertEquals(accelerometerNoiseRootPSD, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(accelerometerNoiseRootPSD * accelerometerNoiseRootPSD, errors.getAccelerometerNoisePSD(), 0.0);
    }

    @Test
    void testGetSetAccelerometerNoisePSD() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getAccelerometerNoisePSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var accelerometerNoisePSD = randomizer.nextDouble(0.0, MAX_VALUE);

        errors.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(accelerometerNoisePSD, errors.getAccelerometerNoisePSD(), ABSOLUTE_ERROR);
        assertEquals(Math.sqrt(accelerometerNoisePSD), errors.getAccelerometerNoiseRootPSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> errors.setAccelerometerNoisePSD(-1.0));
    }

    @Test
    void testGetSetGyroNoiseRootPSD() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getGyroNoisePSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setGyroNoiseRootPSD(gyroNoiseRootPSD);

        // check
        assertEquals(gyroNoiseRootPSD, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD * gyroNoiseRootPSD, errors.getGyroNoisePSD(), 0.0);
    }

    @Test
    void testGetSetGyroNoisePSD() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getGyroNoiseRootPSD(), 0.0);
        assertEquals(0.0, errors.getGyroNoisePSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(0.0, MAX_VALUE);

        errors.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(gyroNoisePSD, errors.getGyroNoisePSD(), ABSOLUTE_ERROR);
        assertEquals(Math.sqrt(gyroNoisePSD), errors.getGyroNoiseRootPSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> errors.setGyroNoisePSD(-1.0));
    }

    @Test
    void testGetSetAccelerometerQuantizationLevel() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getAccelerometerQuantizationLevel(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var level = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setAccelerometerQuantizationLevel(level);

        // check
        assertEquals(level, errors.getAccelerometerQuantizationLevel(), 0.0);
    }

    @Test
    void testGetSetAccelerometerQuantizationLevelAsAcceleration() {
        final var errors = new IMUErrors();

        // check default value
        final var level1 = errors.getAccelerometerQuantizationLevelAsAcceleration();

        assertEquals(0.0, level1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, level1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var level2 = new Acceleration(value, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        errors.setAccelerometerQuantizationLevel(level2);

        // check
        final var level3 = errors.getAccelerometerQuantizationLevelAsAcceleration();
        final var level4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        errors.getAccelerometerQuantizationLevelAsAcceleration(level4);

        assertEquals(level2, level3);
        assertEquals(level2, level4);
    }

    @Test
    void testGetSetGyroQuantizationLevel() {
        final var errors = new IMUErrors();

        // check default value
        assertEquals(0.0, errors.getGyroQuantizationLevel(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var level = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        errors.setGyroQuantizationLevel(level);

        // check
        assertEquals(level, errors.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    void testGyroQuantizationLevelAsAngularSpeed() {
        final var errors = new IMUErrors();

        // check default value
        final var level1 = errors.getGyroQuantizationLevelAsAngularSpeed();

        assertEquals(0.0, level1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, level1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var level2 = new AngularSpeed(value, AngularSpeedUnit.RADIANS_PER_SECOND);

        errors.setGyroQuantizationLevel(level2);

        // check
        final var level3 = errors.getGyroQuantizationLevelAsAngularSpeed();
        final var level4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        errors.getGyroQuantizationLevelAsAngularSpeed(level4);

        assertEquals(level2, level3);
        assertEquals(level2, level4);
    }

    @Test
    void testCopyTo() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final var errors2 = new IMUErrors();

        // copy
        errors.copyTo(errors2);

        // check
        assertArrayEquals(accelerometerBiases, errors2.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors2.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors2.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors2.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors2.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors2.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors2.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors2.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors2.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    void testCopyFrom() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final var errors2 = new IMUErrors();

        // copy
        errors2.copyFrom(errors);

        // check
        assertArrayEquals(accelerometerBiases, errors2.getAccelerometerBiases(), 0.0);
        assertArrayEquals(gyroBiases, errors2.getGyroBiases(), 0.0);
        assertEquals(accelerometerScaleFactorAndCrossCouplingErrors,
                errors2.getAccelerometerScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroScaleFactorAndCrossCouplingErrors, errors2.getGyroScaleFactorAndCrossCouplingErrors());
        assertEquals(gyroGDependenciesBiases, errors2.getGyroGDependentBiases());
        assertEquals(accelerometerNoiseRootPSD, errors2.getAccelerometerNoiseRootPSD(), 0.0);
        assertEquals(gyroNoiseRootPSD, errors2.getGyroNoiseRootPSD(), 0.0);
        assertEquals(accelerometerQuantizationLevel, errors2.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroQuantizationLevel, errors2.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    void testHashCode() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final var errors2 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final var errors3 = new IMUErrors();

        assertEquals(errors1.hashCode(), errors2.hashCode());
        assertNotEquals(errors1.hashCode(), errors3.hashCode());
    }

    @Test
    void testEquals() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final var errors2 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);
        final var errors3 = new IMUErrors();

        //noinspection SimplifiableJUnitAssertion,EqualsWithItself
        assertEquals(errors1, errors1);
        //noinspection SimplifiableJUnitAssertion
        assertEquals(errors1, errors2);
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(errors1, errors3);
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertNotEquals(null, errors1);
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), errors1);
    }

    @Test
    void testClone() throws CloneNotSupportedException, WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final var errors2 = errors1.clone();

        assertEquals(errors1, errors2);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var accelerometerBiases = new double[COMPONENTS];
        randomizer.fill(accelerometerBiases, MIN_VALUE, MAX_VALUE);

        final var gyroBiases = new double[COMPONENTS];
        randomizer.fill(gyroBiases, MIN_VALUE, MAX_VALUE);

        final var accelerometerScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, accelerometerScaleFactorAndCrossCouplingErrors);

        final var gyroScaleFactorAndCrossCouplingErrors = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroScaleFactorAndCrossCouplingErrors);

        final var gyroGDependenciesBiases = new Matrix(COMPONENTS, COMPONENTS);
        Matrix.fillWithUniformRandomValues(MIN_VALUE, MAX_VALUE, gyroGDependenciesBiases);

        final var accelerometerNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroNoiseRootPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var accelerometerQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroQuantizationLevel = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var errors1 = new IMUErrors(accelerometerBiases, gyroBiases,
                accelerometerScaleFactorAndCrossCouplingErrors, gyroScaleFactorAndCrossCouplingErrors,
                gyroGDependenciesBiases, accelerometerNoiseRootPSD, gyroNoiseRootPSD, accelerometerQuantizationLevel,
                gyroQuantizationLevel);

        final var bytes = SerializationHelper.serialize(errors1);
        final var errors2 = SerializationHelper.deserialize(bytes);

        assertEquals(errors1, errors2);
        assertNotSame(errors1, errors2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = IMUErrors.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
