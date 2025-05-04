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

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownHardIronAndFrameMagnetometerCalibratorTest implements 
        RobustKnownHardIronAndFrameMagnetometerCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() {
        // MSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, 
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate5() {
        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate6() {
        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate7() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate8() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate9() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate10() {
        final var qualityScores = new double[4];

        // MSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate11() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate12() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate13() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate14() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate15() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate16() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() {
        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod3() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateWithDefaultMethod4() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod5() {
        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod6() {
        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                this);
        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod7() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                true);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod8() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                true, this);

        // check
        assertInstanceOf(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, final float progress) {
        // no action needed
    }
}
