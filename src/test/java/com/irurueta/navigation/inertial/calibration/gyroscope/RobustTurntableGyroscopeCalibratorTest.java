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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustTurntableGyroscopeCalibratorTest implements
        RobustTurntableGyroscopeCalibratorListener {

    private static final double ROTATION_RATE = Math.PI / 2.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testCreate1() {
        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
    }

    @Test
    public void testCreate2() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate3() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate4() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate5() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate6() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate7() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias,  calibrator.getAccelerometerBias(),0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate8() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate9() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testCreate10() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate11() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate12() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate13() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate14() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate15() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate16() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate17() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate18() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate19() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate20() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate21() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate22() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate23() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate24() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate25() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate26() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate27() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate28() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate29() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate30() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate31() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate32() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate33() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate34() {
        final double[] qualityScores = new double[10];

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    public void testCreate35() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate36() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate37() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate38() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(),
                0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate39() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate40() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate41() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate42() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate43() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, true,
                true, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate44() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, true,
                true, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate45() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate46() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate47() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(calibrator.getInitialGg(), initialGg);

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate48() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate49() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate50() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate51() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate52() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate53() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate54() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate55() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate56() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate57() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate58() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate59() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate60() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate61() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, true,
                true, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate62() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, true,
                true, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate63() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate64() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate65() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreate66() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate67() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate68() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreate69() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreate70() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        // RANSAC
        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores,
                position, ROTATION_RATE, TIME_INTERVAL, measurements, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(calibrator instanceof LMedSRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustTurntableGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustTurntableGyroscopeCalibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod1() {
        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    public void testCreateWithDefaultMethod2() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod3() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod4() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod5() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod6() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod7() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod8() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod9() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod10() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, true, true,
                initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod11() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, true, true,
                initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod12() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod13() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod14() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod15() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod16() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod17() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod18() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod19() throws WrongSizeException {
        final ECEFPosition position = new ECEFPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod20() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod21() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod22() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod23() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod24() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod25() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod26() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod27() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod28() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, true, true,
                initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod29() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, true, true,
                initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod30() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod31() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod32() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    public void testCreateWithDefaultMethod33() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod34() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod35() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final double[] initialBias = new double[3];
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final double[] accelerometerBias = new double[3];
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testCreateWithDefaultMethod36() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    public void testCreateWithDefaultMethod37() throws WrongSizeException {
        final NEDPosition position = new NEDPosition();
        final List<StandardDeviationBodyKinematics> measurements = Collections.emptyList();
        final Matrix initialBias = new Matrix(3, 1);
        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        final Matrix accelerometerBias = new Matrix(3, 1);
        final Matrix accelerometerMa = new Matrix(3, 3);

        final RobustTurntableGyroscopeCalibrator calibrator = RobustTurntableGyroscopeCalibrator.create(position,
                ROTATION_RATE, TIME_INTERVAL, measurements, false, false,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustTurntableGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustTurntableGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(final RobustTurntableGyroscopeCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(final RobustTurntableGyroscopeCalibrator calibrator, final float progress) {
        // no action needed
    }
}
