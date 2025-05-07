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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownHardIronPositionAndInstantMagnetometerCalibratorTest implements
        RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() {
        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate5() {
        final var magneticModel = new WorldMagneticModel();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    void testCreate6() {
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate7() {
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate8() {
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate9() {
        final var position = new NEDPosition();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
    }

    @Test
    void testCreate10() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate11() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements, 
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate12() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate13() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate14() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate15() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate16() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate17() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate18() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(calibrator.getHardIronMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate19() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate20() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate21() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate22() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate23() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate24() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate25() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate26() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    void testCreate27() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate28() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate29() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate30() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate31() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(calibrator.getHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate32() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate33() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate34() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate35() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate36() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate37() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate38() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate39() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate40() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate41() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate42() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate43() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate44() {
        final var qualityScores = new double[10];

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate45() {
        final var qualityScores = new double[10];
        final var magneticModel = new WorldMagneticModel();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                magneticModel, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate46() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate47() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate48() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores,
                initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate49() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate50() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate51() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate52() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate53() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate54() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate55() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate56() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate57() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate58() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getHardIronMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate59() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate60() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate61() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate62() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate63() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate64() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate65() {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate66() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate67() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate68() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate69() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate70() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate71() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate72() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate73() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate74() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate75() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate76() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate77() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate78() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate79() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate80() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate81() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate82() {
        final var qualityScores = new double[10];
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() {
        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateWithDefaultMethod4() {
        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod5() {
        final var magneticModel = new WorldMagneticModel();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(magneticModel);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    void testCreateWithDefaultMethod6() {
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreateWithDefaultMethod7() {
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreateWithDefaultMethod8() {
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreateWithDefaultMethod9() {
        final var position = new NEDPosition();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
    }

    @Test
    void testCreateWithDefaultMethod10() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateWithDefaultMethod11() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod12() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod13() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod14() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreateWithDefaultMethod15() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod16() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreateWithDefaultMethod17() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod18() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreateWithDefaultMethod19() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod20() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreateWithDefaultMethod21() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod22() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreateWithDefaultMethod23() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod24() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreateWithDefaultMethod25() {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod26() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    void testCreateWithDefaultMethod27() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateWithDefaultMethod28() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod29() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod30() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod31() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreateWithDefaultMethod32() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod33() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreateWithDefaultMethod34() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod35() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreateWithDefaultMethod36() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod37() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreateWithDefaultMethod38() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod39() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreateWithDefaultMethod40() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod41() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreateWithDefaultMethod42() {
        final var nedPosition = new NEDPosition();
        final var nedVelocity = new NEDVelocity();
        final var ecefVelocity = new ECEFVelocity();
        final var position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronPositionAndInstantMagnetometerCalibrator.create(position,
                measurements, true, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static double[] generateHardIron() {
        final var randomizer = new UniformRandomizer();

        final var result = new double[3];
        randomizer.fill(result);

        return result;
    }

    private static Matrix generateHardIronMatrix() {
        return Matrix.newFromArray(generateHardIron());
    }

    private static Matrix generateMm() {
        try {
            return Matrix.createWithUniformRandomValues(
                    3, 3, -1.0, 1.0);
        } catch (WrongSizeException ignore) {
            return null;
        }
    }
}
