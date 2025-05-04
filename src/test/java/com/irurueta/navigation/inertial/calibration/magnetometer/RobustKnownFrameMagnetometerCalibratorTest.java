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

class RobustKnownFrameMagnetometerCalibratorTest implements RobustKnownFrameMagnetometerCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() {
        // MSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate3() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreate4() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate5() {
        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate6() {
        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate7() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreate8() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate9() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate10() {
        final var qualityScores = new double[4];

        // MSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate11() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate12() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate13() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate14() {
        final var qualityScores = new double[4];

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate15() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate16() {
        final var qualityScores = new double[4];
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        // RANSAC
        var calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, 
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create();

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() {
        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(this);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod3() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testCreateWithDefaultMethod4() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod5() {
        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(true);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod6() {
        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod7() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testCreateWithDefaultMethod8() {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                this);

        // check
        assertInstanceOf(LMedSRobustKnownFrameMagnetometerCalibrator.class, calibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownFrameMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownFrameMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(final RobustKnownFrameMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(final RobustKnownFrameMagnetometerCalibrator calibrator,
                                          final float progress) {
        // no action needed
    }
}
