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

public class RobustKnownHardIronAndFrameMagnetometerCalibratorTest implements
        RobustKnownHardIronAndFrameMagnetometerCalibratorListener {

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // MSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreate4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate5() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate6() {
        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate9() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate10() {
        final double[] qualityScores = new double[4];

        // MSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate11() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate12() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate13() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate14() {
        final double[] qualityScores = new double[4];

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate15() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate16() {
        final double[] qualityScores = new double[4];
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                        true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownHardIronAndFrameMagnetometerCalibrator.create(qualityScores, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(true, this);
        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                RobustKnownHardIronAndFrameMagnetometerCalibrator.create(measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator);
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
