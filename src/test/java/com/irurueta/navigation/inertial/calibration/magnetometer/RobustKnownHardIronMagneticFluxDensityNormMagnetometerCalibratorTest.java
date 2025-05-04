/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest implements
        RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() {
        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate5() {
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate6() {
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(calibrator.getHardIronMatrix(), hardIron);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate7() {
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate8() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate9() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate10() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate11() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate12() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate13() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate14() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate15() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate16() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate17() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate18() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate19() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate20() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, hardIron,
                initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate21() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate22() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, hardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate23() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
    }

    @Test
    void testCreate24() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate25() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate26() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate27() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate28() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate29() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate30() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate31() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate32() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate33() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate34() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate35() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
    }

    @Test
    void testCreate36() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate37() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate38() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate39() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
    }

    @Test
    void testCreate40() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate41() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate42() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate43() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate44() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate45() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate46() {
        final var qualityScores = new double[10];

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate47() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate48() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate49() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, hardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate50() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate51() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate52() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate53() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate54() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate55() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate56() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate57() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate58() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate59() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate60() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getHardIronMatrix(), hardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate61() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate62() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, hardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate63() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate64() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, hardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate65() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate66() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate67() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate68() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate69() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate70() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate71() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate72() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate73() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate74() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate75() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate76() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate77() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate78() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate79() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate80() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(calibrator.getHardIronMatrix(), hardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate81() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate82() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate83() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate84() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate85() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate86() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() {
        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod4() {
        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod5() {
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod6() {
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod7() {
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(hardIron, 
                initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod8() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod9() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod10() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod11() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod12() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod13() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod14() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod15() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod16() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod17() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod18() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod19() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod20() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod21() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod22() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod23() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod24() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod25() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod26() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod27() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod28() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod29() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod30() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod31() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod32() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod33() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod34() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod35() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod36() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod37() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod38() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod39() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod40() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod41() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod42() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod43() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod44() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm, 
                this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod45() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod46() {
        final var qualityScores = new double[10];

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod47() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod48() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod49() {
        final var qualityScores = new double[10];
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod50() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod51() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod52() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod53() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod54() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod55() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod56() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod57() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod58() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod59() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod60() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod61() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod62() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod63() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod64() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, 
                measurements, true, hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod65() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod66() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod67() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod68() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod69() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod70() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod71() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod72() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod73() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod74() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod75() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod76() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod77() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod78() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIron();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod79() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod80() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod81() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod82() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod83() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod84() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod85() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod86() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var hardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, initialMm, 
                this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(hardIron, calibrator.getHardIronMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static double[] generateHardIron() {
        final var randomizer = new UniformRandomizer();

        final var result = new double[3];
        randomizer.fill(result);

        return result;
    }

    private static double generateGroundTruthMagneticFluxDensityNorm() {
        return new UniformRandomizer().nextDouble();
    }

    private static Matrix generateHardIronMatrix() {
        return Matrix.newFromArray(generateHardIron());
    }

    private static Matrix generateMm() {
        try {
            return Matrix.createWithUniformRandomValues(3, 3, -1.0, 1.0);
        } catch (WrongSizeException ignore) {
            return null;
        }
    }
}