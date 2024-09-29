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
import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RobustKnownPositionAndInstantMagnetometerCalibratorTest implements
        RobustKnownPositionAndInstantMagnetometerCalibratorListener {

    private static final double ABSOLUTE_ERROR = 1e-5;

    @Test
    public void testCreate1() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
    }

    @Test
    public void testCreate2() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreate4() {
        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    public void testCreate6() {
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreate7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreate8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreate9() {
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
    }

    @Test
    public void testCreate10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreate11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate12() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate13() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate14() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreate15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate16() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreate17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate18() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreate19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate20() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreate21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate22() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreate23() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate24() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreate25() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(
                position, measurements, true, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(calibrator.getNedPosition(), position);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate26() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    public void testCreate27() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreate28() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate29() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreate30() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate31() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreate32() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate33() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreate34() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate35() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreate36() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate37() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreate38() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate39() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreate40() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate41() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreate42() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate43() {
        final double[] qualityScores = new double[10];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate44() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, true,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, true,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate45() {
        final double[] qualityScores = new double[10];
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, magneticModel,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate46() {
        final double[] qualityScores = new double[10];
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate47() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate48() {
        final double[] qualityScores = new double[10];
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron, initialMm,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, initialHardIron,
                initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate49() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate50() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate51() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate52() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate53() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                measurements, true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate54() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate55() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate56() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate57() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate58() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate59() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate60() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate61() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate62() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate63() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(calibrator.getListener(), this);
        assertSame(calibrator.getQualityScores(), qualityScores);
    }

    @Test
    public void testCreate64() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate65() {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate66() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate67() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate68() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate69() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate70() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate71() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate72() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(calibrator.getInitialHardIron(), initialHardIron, 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate73() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate74() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate75() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate76() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate77() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate78() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate79() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate80() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate81() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, initialMm, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate82() {
        final double[] qualityScores = new double[10];
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        // RANSAC
        RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                        true, initialHardIron, initialMm, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownPositionAndInstantMagnetometerCalibrator.create(qualityScores, position, measurements,
                true, initialHardIron, initialMm, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create();

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod2() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod3() {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreateWithDefaultMethod4() {
        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod5() {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(magneticModel);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    public void testCreateWithDefaultMethod6() {
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod7() {
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreateWithDefaultMethod8() {
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreateWithDefaultMethod9() {
        final NEDPosition position = new NEDPosition();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
    }

    @Test
    public void testCreateWithDefaultMethod10() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreateWithDefaultMethod11() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod12() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod13() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod14() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod15() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod16() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod17() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod18() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreateWithDefaultMethod19() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod20() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreateWithDefaultMethod21() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod22() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreateWithDefaultMethod23() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod24() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreateWithDefaultMethod25() {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getNedPosition());
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod26() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
    }

    @Test
    public void testCreateWithDefaultMethod27() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testCreateWithDefaultMethod28() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod29() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements,
                        true);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testCreateWithDefaultMethod30() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod31() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod32() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod33() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
    }

    @Test
    public void testCreateWithDefaultMethod34() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final double[] initialHardIron = generateHardIron();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertArrayEquals(initialHardIron, calibrator.getInitialHardIron(), 0.0);
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod35() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreateWithDefaultMethod36() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod37() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
    }

    @Test
    public void testCreateWithDefaultMethod38() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod39() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreateWithDefaultMethod40() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, initialHardIron,
                        initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod41() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
    }

    @Test
    public void testCreateWithDefaultMethod42() {
        final NEDPosition nedPosition = new NEDPosition();
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        final ECEFPosition position = new ECEFPosition();

        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity, position, ecefVelocity);

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final Matrix initialHardIron = generateHardIronMatrix();
        final Matrix initialMm = generateMm();

        final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                RobustKnownPositionAndInstantMagnetometerCalibrator.create(position, measurements, true,
                        initialHardIron, initialMm, this);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownPositionAndInstantMagnetometerCalibrator);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getEcefPosition().equals(position, ABSOLUTE_ERROR));
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertEquals(initialHardIron, calibrator.getInitialHardIronAsMatrix());
        assertEquals(initialMm, calibrator.getInitialMm());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownPositionAndInstantMagnetometerCalibrator calibrator, final float progress) {
        // no action needed
    }

    private static double[] generateHardIron() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double[] result = new double[3];
        randomizer.fill(result);

        return result;
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
