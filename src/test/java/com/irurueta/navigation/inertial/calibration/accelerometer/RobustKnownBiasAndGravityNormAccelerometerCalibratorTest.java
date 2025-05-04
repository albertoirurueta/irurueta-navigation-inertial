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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownBiasAndGravityNormAccelerometerCalibratorTest implements
        RobustKnownBiasAndGravityNormAccelerometerCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
    }

    @Test
    void testCreate2() {
        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate5() {
        final var initialBias = new double[3];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate6() throws WrongSizeException {
        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate7() throws WrongSizeException {
        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithGaussianRandomValues(
                3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate8() {
        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        // RANSAC
        RobustKnownBiasAndGravityNormAccelerometerCalibrator calibrator =
                RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
    }

    @Test
    void testCreate9() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate10() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate11() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate12() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate13() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate14() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate15() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate16() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate17() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate18() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate21() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate22() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(
                gravityNorm, measurements, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate25() {
        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
    }

    @Test
    void testCreate26() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate27() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate28() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate29() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(
                gravityNorm, measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate30() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate31() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate32() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate33() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate36() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate37() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, initialBias,
                initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate40() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate41() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements, 
                true, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate42() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate43() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate44() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate45() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate46() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate47() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate48() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate49() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate50() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate51() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate52() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate53() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate54() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(
                3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(
                3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate55() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate56() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate57() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate58() {
        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
    }

    @Test
    void testCreate59() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate60() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate61() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate62() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate63() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate64() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate65() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreate66() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate67() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate68() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate69() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreate70() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate71() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(
                qualityScores, gravityNorm, measurements, initialBias,
                initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate72() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate73() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm,
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreate74() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var qualityScores = new double[13];

        // RANSAC
        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.RANSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(qualityScores, gravityNorm, 
                measurements, true, initialBias, initialMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault1() {
        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateDefault2() {
        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(this);

        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault3() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var calibrator =
                RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(measurements);

        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateDefault4() {
        final var calibrator =
                RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateDefault5() {
        final var initialBias = new double[3];
        final var randomizer = new UniformRandomizer(new Random());
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreateDefault6() throws WrongSizeException {
        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreateDefault7() throws WrongSizeException {
        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithGaussianRandomValues(
                3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(initialBias, initialMa);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreateDefault8() {
        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
    }

    @Test
    void testCreateDefault9() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateDefault10() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault11() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateDefault12() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault13() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreateDefault14() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault15() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreateDefault16() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault17() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreateDefault18() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreateDefault20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault21() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, initialMa);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreateDefault22() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, initialMa, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault23() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreateDefault24() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = randomizer.nextDouble();

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault25() {
        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
    }

    @Test
    void testCreateDefault26() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateDefault27() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault28() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateDefault29() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault30() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreateDefault31() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault32() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
    }

    @Test
    void testCreateDefault33() {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialBias = new double[3];
        randomizer.fill(initialBias);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault34() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreateDefault35() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault36() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
    }

    @Test
    void testCreateDefault37() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault38() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, initialMa);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreateDefault39() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                initialBias, initialMa, this);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefault40() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
    }

    @Test
    void testCreateDefault41() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        final var randomizer = new UniformRandomizer(new Random());
        final var gravityNorm = new Acceleration(randomizer.nextDouble(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final var initialBias = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);
        final var initialMa = Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);

        final var calibrator = RobustKnownBiasAndGravityNormAccelerometerCalibrator.create(gravityNorm, measurements,
                true, initialBias, initialMa, this);
        // check
        assertInstanceOf(LMedSRobustKnownBiasAndGravityNormAccelerometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(gravityNorm, calibrator.getGroundTruthGravityNormAsAcceleration());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMa, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasAndGravityNormAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasAndGravityNormAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndGravityNormAccelerometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndGravityNormAccelerometerCalibrator calibrator, final float progress) {
        // no action needed
    }
}
