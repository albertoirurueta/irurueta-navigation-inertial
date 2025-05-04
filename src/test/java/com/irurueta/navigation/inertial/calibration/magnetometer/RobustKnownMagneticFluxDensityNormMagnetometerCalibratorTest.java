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

class RobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest implements
        RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() {
        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate5() {
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate6() {
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate7() {
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate8() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate9() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate10() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate11() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate12() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate13() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate14() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate15() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate16() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate17() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate18() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate19() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate20() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate21() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate22() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, 
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements, true,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate23() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
    }

    @Test
    void testCreate24() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate25() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate26() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate27() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate28() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate29() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate30() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate31() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate32() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate33() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate34() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate35() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    void testCreate36() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate37() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate38() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate39() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    void testCreate40() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate41() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate42() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate43() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    void testCreate44() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(groundTruthMagneticFluxDensityNorm,
                measurements, true, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate45() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate46() {
        final var qualityScores = new double[10];

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate47() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate48() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate49() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate50() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate51() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate52() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate53() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate54() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate55() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate56() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate57() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate58() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate59() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate60() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate61() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate62() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate63() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate64() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate65() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate66() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate68() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate69() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate70() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate71() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate72() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate76() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate77() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate78() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate79() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate80() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate81() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate82() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate83() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate84() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate85() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate86() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() {
        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod3() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod4() {
        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod5() {
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod6() {
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod7() {
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(initialHardIron,
                initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod8() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod9() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod10() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod11() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod12() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod13() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod14() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod15() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod16() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod17() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod18() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod19() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod20() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod21() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod22() {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(measurements,
                true, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod23() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod24() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod25() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod26() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod27() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod28() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod29() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod30() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod31() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod32() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod34() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod35() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod36() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod37() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        
        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod38() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod39() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod40() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod41() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod42() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod43() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod44() {
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        // RANSAC
        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod45() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod46() {
        final var qualityScores = new double[10];

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod47() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod48() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod49() {
        final var qualityScores = new double[10];
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod50() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod51() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod52() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores, measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod54() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod55() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod56() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod57() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod58() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod59() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod60() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod61() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod62() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod63() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod64() {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                measurements, true, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod65() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod66() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod68() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod69() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod70() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod71() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod72() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
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
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod76() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod77() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod78() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIron();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod79() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod80() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod81() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod82() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod83() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod84() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
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
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod86() {
        final var qualityScores = new double[10];
        final var groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var initialHardIron = generateHardIronMatrix();
        final var initialMm = generateMm();

        final var calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, initialHardIron, initialMm,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.class, calibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Override
    public void onCalibrateStart(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
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