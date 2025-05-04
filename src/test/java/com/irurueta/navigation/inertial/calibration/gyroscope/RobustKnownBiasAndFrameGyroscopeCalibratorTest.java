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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownBiasAndFrameGyroscopeCalibratorTest implements
        RobustKnownBiasAndFrameGyroscopeCalibratorListener {

    private static final double DEG_TO_RAD = 0.01745329252;

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);

        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // test create with measurements and method
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates and method
        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with measurements, bias, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, 
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, 
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, 
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias coordinates, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test constructor with bias coordinates as angular speed
        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // test create with bias coordinates as angular speed, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates as angular speed and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // test create with measurements, bias coordinates as angular speed, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates as angular speed, common axis used and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates angular speed, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates as angular speed, common axis
        // used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias coordinates as angular speed, common axis
        // used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias array and method
        final var bias = bg.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with measurements, bias coordinates array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with bias coordinates array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias coordinates array, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias coordinates array, common axis used and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // test create with bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // test create with measurements, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with bias matrix, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, bias matrix, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and method

        // RANSAC
        final var qualityScores = new double[RobustKnownBiasAndFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measures, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, bias coordinates, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, measurements, bias coordinates and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, measurements, bias coordinates, listener and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        // test create with quality scores, bias coordinates, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias coordinates, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias coordinates,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates, common axis
        // used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, biasX, biasY, biasZ,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias coordinates as angular speed and
        // method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // test create with quality scores, bias coordinates as angular speed, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates as angular
        // speed and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        // test create with quality scores, measurements, bias coordinates as angular speed,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias coordinates as angular speed,
        // common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias coordinates as angular speed,
        // common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bx, by, bz, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates as angular
        // speed, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias coordinates as angular
        // speed, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias coordinates as angular
        // speed, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bx, by, bz,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with quality scores, bias array, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias array and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // test create with quality scores, measurements, bias array,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias array, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bias, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias array, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias array, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bias,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // test create with quality scores, bias matrix, listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias matrix and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        // test create with quality scores, measurements, bias matrix, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, bias matrix, common axis used and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores,
                bg, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, bias matrix, common axis used, listener
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, bg, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, bias matrix, common axis used
        // and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, bias matrix, common axis used,
        // listener and method

        // RANSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(qualityScores, measurements, bg,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateDefaultMethod() {
        var calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var bg = generateBg();
        final var biasX = bg.getElementAtIndex(0);
        final var biasY = bg.getElementAtIndex(1);
        final var biasZ = bg.getElementAtIndex(2);
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(biasX, biasY, biasZ, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, biasX, biasY, biasZ,
                true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var bx = new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by = new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz = new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bx, by, bz, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bx, by, bz, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bx, calibrator.getBiasAngularSpeedX());
        assertEquals(by, calibrator.getBiasAngularSpeedY());
        assertEquals(bz, calibrator.getBiasAngularSpeedZ());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        final var bias = bg.getBuffer();
        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bias, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bias, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(bg, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());

        calibrator = RobustKnownBiasAndFrameGyroscopeCalibrator.create(measurements, bg, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(bg, calibrator.getBiasAsMatrix());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndFrameGyroscopeCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }
}
