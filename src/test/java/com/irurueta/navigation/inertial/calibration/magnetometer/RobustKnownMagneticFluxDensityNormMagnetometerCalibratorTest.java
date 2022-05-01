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
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest implements
        RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate4() {
        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate5() {
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate6() {
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate7() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate8() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate9() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate10() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate11() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate12() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate13() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate14() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate15() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate16() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate17() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate18() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate19() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate20() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate21() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate22() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, initialMm,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                measurements, true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate23() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
    }

    @Test
    public void testCreate24() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate25() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testCreate26() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate27() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate28() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate29() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate30() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate31() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate32() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        measurements, true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate33() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate34() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate35() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
    }

    @Test
    public void testCreate36() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate37() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate38() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate39() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
    }

    @Test
    public void testCreate40() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate41() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate42() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate43() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
    }

    @Test
    public void testCreate44() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate45() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate46() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate47() {
        final double[] qualityScores = new double[10];
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate48() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate49() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, initialHardIron, initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate50() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate51() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate52() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate53() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate54() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate55() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate56() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate57() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate58() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate59() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate60() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate61() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate62() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate63() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate64() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate65() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate66() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate67() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate68() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate69() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate70() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate71() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate72() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate73() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate74() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        measurements, true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate75() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate76() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate77() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate78() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate79() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate80() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate81() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate82() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, true,
                initialHardIron, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate83() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate84() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                initialHardIron, initialMm, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate85() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate86() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                true, initialHardIron, initialMm,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod9() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod10() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod11() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod12() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod13() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod14() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod15() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod16() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod17() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod18() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod19() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod20() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod21() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod22() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        measurements, true, initialHardIron, initialMm,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod23() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod24() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod25() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod26() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod27() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod28() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod29() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod30() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod31() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod32() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm,
                        measurements, true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod33() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod34() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod35() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod36() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod37() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        
        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod38() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod39() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod40() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod41() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod42() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod43() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod44() {
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod45() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod46() {
        final double[] qualityScores = new double[10];

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod47() {
        final double[] qualityScores = new double[10];
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod48() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod49() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod50() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod51() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod52() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod53() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod54() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod55() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod56() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod57() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod58() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod59() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod60() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod61() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod62() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod63() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod64() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, measurements, true, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod65() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod66() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod67() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod68() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod69() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod70() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod71() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod72() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod73() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod74() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm,
                        measurements, true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod75() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod76() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(calibrator.getListener(), this);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod77() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod78() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod79() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod80() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod81() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod82() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod83() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod84() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod85() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod86() {
        final double[] qualityScores = new double[10];
        final double groundTruthMagneticFluxDensityNorm = generateGroundTruthMagneticFluxDensityNorm();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                RobustKnownMagneticFluxDensityNormMagnetometerCalibrator.create(
                        qualityScores, groundTruthMagneticFluxDensityNorm, measurements,
                        true, initialHardIron, initialMm,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(calibrator.getInitialHardIronAsMatrix(), initialHardIron);
        assertEquals(calibrator.getInitialMm(), initialMm);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());
        assertEquals(RobustEstimatorMethod.LMedS, calibrator.getMethod());
    }

    @Override
    public void onCalibrateStart(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
    }

    private double[] generateHardIron() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());

        final double[] result = new double[3];
        randomizer.fill(result);

        return result;
    }

    private double generateGroundTruthMagneticFluxDensityNorm() {
        return new UniformRandomizer().nextDouble();
    }

    private Matrix generateHardIronMatrix() {
        return Matrix.newFromArray(generateHardIron());
    }

    private Matrix generateMm() {
        try {
            return Matrix.createWithUniformRandomValues(
                    3, 3, -1.0, 1.0);
        } catch (WrongSizeException ignore) {
            return null;
        }
    }
}