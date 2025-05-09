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
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownBiasAndFrameAccelerometerCalibratorTest implements 
        RobustKnownBiasAndFrameAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);

        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // test create with measurements and method
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(this, calibrator.getListener());

        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates and method
        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with measurements, bias, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ, 
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test constructor with bias coordinates as acceleration
        final var bax = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // test create with bias coordinates as acceleration, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates as acceleration and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // test create with measurements, bias coordinates as acceleration, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates as acceleration, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates as acceleration, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates as acceleration, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias coordinates as acceleration, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias array and method
        final var bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with measurements, bias coordinates array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates array, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // test create with bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // test create with measurements, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and method

        // RANSAC
        final var qualityScores = new double[RobustKnownBiasAndFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, measurements, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias coordinates, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias coordinates, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements,
                biasX, biasY, biasZ, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates as acceleration and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // test create with quality scores, bias coordinates as acceleration, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates as acceleration
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        // test create with quality scores, measurements, bias coordinates as acceleration,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates as acceleration, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias coordinates as acceleration,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates as acceleration,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias coordinates as acceleration,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(
                qualityScores, measurements, bax, bay, baz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bax, bay, baz,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with quality scores, bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with quality scores, measurements, bias array,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias array,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias array, common axis used
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // test create with quality scores, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // test create with quality scores, measurements, bias matrix, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias matrix, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, ba, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias matrix, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias matrix,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(qualityScores, measurements, ba,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefaultMethod() {
        var calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());


        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var ba = generateBa();
        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(biasX, biasY, biasZ, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var bax = new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay = new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz = new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bax, bay, baz, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bax, bay, baz,
                true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bax, calibrator.getBiasXAsAcceleration());
        assertEquals(bay, calibrator.getBiasYAsAcceleration());
        assertEquals(baz, calibrator.getBiasZAsAcceleration());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var bias = ba.getBuffer();
        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(bias, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, bias, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(ba, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameAccelerometerCalibrator.create(measurements, ba, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndFrameAccelerometerCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }
}
