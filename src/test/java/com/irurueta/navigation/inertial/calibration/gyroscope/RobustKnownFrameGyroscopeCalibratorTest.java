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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownFrameGyroscopeCalibratorTest implements RobustKnownFrameGyroscopeCalibratorListener {

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var calibrator = RobustKnownFrameGyroscopeCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);

        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // test create with measurements and method

        // RANSAC
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this, 
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores and method
        final var qualityScores = new double[RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // test create with quality scores, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        // test create with listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // test create with measurements and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());

        // test create with quality scores, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measurements, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and default
        // method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener
        // and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores, measurements, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownFrameGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownFrameGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(final RobustKnownFrameGyroscopeCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(final RobustKnownFrameGyroscopeCalibrator calibrator, final float progress) {
        // no action needed
    }
}
