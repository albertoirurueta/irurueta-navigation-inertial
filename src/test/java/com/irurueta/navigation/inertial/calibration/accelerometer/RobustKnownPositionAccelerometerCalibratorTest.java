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
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownPositionAccelerometerCalibratorTest implements RobustKnownPositionAccelerometerCalibratorListener {

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testCreateWithMethod() throws WrongSizeException {

        // create 1

        // RANSAC
        var calibrator = RobustKnownPositionAccelerometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);

        // create 2

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // create 3
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // create 4

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // create 5
        final var ba = generateBa();
        final var bias = ba.getBuffer();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 6

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 7
        final var ma = generateMa();

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 8
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // create 9

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 10

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 11

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 12

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 13

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 14

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 15

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 16

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 17

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 18

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 19

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 20

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 21

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 22

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 23

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 24

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 25

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // create 26

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // create 27

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 28

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 29

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 30

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 31

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 32

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 33

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 34

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 35

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 36

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 37

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 38

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 39

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 40

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 41

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 42
        final var qualityScores = new double[13];

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // create 43

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 44

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 45

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 46

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 47

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 48

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 49

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 50

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 51

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 52

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 53

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 54

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 55

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements, ba,
                ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 56

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 57

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, ecefPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 58

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // create 59

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // create 60

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 61

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 62

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 63

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, bias,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 64

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 65

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialBias(), bias, 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROmedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, bias, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 66

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 67

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 68

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 69

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 70

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 71

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements, ba, ma,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 72

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 73

        // RANSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(qualityScores, nedPosition, measurements,
                true, ba, ma, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownPositionAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod() throws WrongSizeException {

        // create 1
        var calibrator = RobustKnownPositionAccelerometerCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        // create 2
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // create 3
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // create 4
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 5
        final var ba = generateBa();
        final var bias = ba.getBuffer();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 6
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // calibrate 7
        final var ma = generateMa();
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // calibrate 8
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);
        final var nedVelocity = new NEDVelocity();
        final var ecefPosition = new ECEFPosition();
        final var ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, ecefPosition, ecefVelocity);

        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());

        // calibrate 9
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // calibrate 10
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // calibrate 11
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // create 12
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // create 13
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 14
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 15
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // create 16
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // create 17
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(calibrator.getInitialBiasAsMatrix(), ba);

        // create 18
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 19
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 20
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 21
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 22
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, ba, ma, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 23
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 24
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(ecefPosition, measurements, true,
                ba, ma, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(ecefPosition, calibrator.getEcefPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 25
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));

        // calibrate 26
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // calibrate 27
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // calibrate 28
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // calibrate 29
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // calibrate 30
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // calibrate 31
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // calibrate 32
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);

        // calibrate 33
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                bias, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(bias, calibrator.getInitialBias(), 0.0);
        assertSame(this, calibrator.getListener());

        // calibrate 34
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // calibrate 35
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 36
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());

        // create 37
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertSame(this, calibrator.getListener());

        // create 38
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 39
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, ba, ma, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());

        // create 40
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());

        // create 41
        calibrator = RobustKnownPositionAccelerometerCalibrator.create(nedPosition, measurements, true,
                ba, ma, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(ba, calibrator.getInitialBiasAsMatrix());
        assertEquals(ma, calibrator.getInitialMa());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownPositionAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownPositionAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAccelerometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAccelerometerCalibrator calibrator, final float progress) {
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
