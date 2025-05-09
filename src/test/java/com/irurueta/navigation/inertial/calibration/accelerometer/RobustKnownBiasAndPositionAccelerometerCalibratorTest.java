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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownBiasAndPositionAccelerometerCalibratorTest implements 
        RobustKnownBiasAndPositionAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testCreateWithMethod() throws WrongSizeException {
        // create 1

        // RANSAC
        var calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);

        // create 2

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // create 3
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // create 4

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // create 5
        final var ba = generateBa();
        final var bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 6

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 7
        final var ma = generateMa();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 8
        final var ecefPosition = new ECEFPosition();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // create 9

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 10

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 11

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, 
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, 
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 12

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 13

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 14

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 15

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, 
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // chekc
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 16

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, 
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 17

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 18

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 19

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 20

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 21

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 22

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 23

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 24

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 25
        final var nedPosition = new NEDPosition();

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // create 26

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // create 27

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 28

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 29

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 30

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 31

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 32

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // chekc
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 33

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 34

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 35

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 36

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 37

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 38

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 39

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 40

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 41

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 42
        final var qualityScores = new double[10];

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 43

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 44

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 45

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 46

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 47

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 48

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 49

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 50

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 51

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 52

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 53

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 54

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 55

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 56

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 57

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 58

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // create 59

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 60

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 69

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 70

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 71

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 72

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 73

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 74

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 75

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 76

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 77

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 78

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 79

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 80

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 81

        // RANSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        assertInstanceOf(RANSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        assertInstanceOf(LMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        assertInstanceOf(MSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        assertInstanceOf(PROSACRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        assertInstanceOf(PROMedSRobustKnownBiasAndPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod() throws WrongSizeException {
        // create 1
        var calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        // create 2
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // create 3
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // create 4
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 5
        final var ba = generateBa();
        final var bias = ba.getBuffer();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 6
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 7
        final var ma = generateMa();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 8
        final var ecefPosition = new ECEFPosition();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // create 9
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 10
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 11
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 12
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 13
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 14
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 15
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 16
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 17
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 18
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 19
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 20
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 21
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 22
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 23
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma);

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 24
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                true, ba, ma, this);

        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 25
        final var nedPosition = new NEDPosition();
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // create 26
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // create 27
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 28
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 29
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 30
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 31
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 32
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);

        // create 33
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 34
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 35
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 36
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());

        // create 37
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 38
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 39
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 40
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma);

        //check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 41
        calibrator = RobustKnownBiasAndPositionAccelerometerCalibrator.create(nedPosition, measurements,
                true, ba, ma, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
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
}
