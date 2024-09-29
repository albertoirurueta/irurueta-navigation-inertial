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
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownFrameMagnetometerCalibratorTest implements
        RobustKnownFrameMagnetometerCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // MSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreate4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate5() {
        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate6() {
        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate9() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate10() {
        final double[] qualityScores = new double[4];

        // MSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate11() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate12() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate13() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                qualityScores, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate14() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate15() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate16() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores,
                measurements, true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameMagnetometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownFrameMagnetometerCalibrator calibrator = RobustKnownFrameMagnetometerCalibrator.create(
                measurements, true, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameMagnetometerCalibrator);
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
