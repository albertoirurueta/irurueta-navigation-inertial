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

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownFrameAccelerometerCalibratorTest implements RobustKnownFrameAccelerometerCalibratorListener {

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var calibrator = RobustKnownFrameAccelerometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);

        // test create with listener and method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(this, calibrator.getListener());

        // test create with measurements and method

        // RANSAC
        final var measurements = Collections.<StandardDeviationFrameBodyKinematics>emptyList();
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this, 
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores and method
        final var qualityScores = new double[RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // test create with quality scores, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownFrameAccelerometerCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());

        // test create with listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());

        // test create with measurements and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with measurements, common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with measurements, common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());

        // test create with quality scores, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());

        // test create with quality scores, measurements, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, true, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // test create with quality scores, measurements, common axis used and default
        // method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // test create with quality scores, measurements, common axis used, listener
        // and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores, measurements, true,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertNull(calibrator.getQualityScores());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownFrameAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownFrameAccelerometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownFrameAccelerometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownFrameAccelerometerCalibrator calibrator, final float progress) {
        // no action needed
    }
}
