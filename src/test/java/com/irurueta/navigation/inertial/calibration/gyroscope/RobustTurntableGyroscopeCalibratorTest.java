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
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustTurntableGyroscopeCalibratorTest implements RobustTurntableGyroscopeCalibratorListener {

    private static final double ROTATION_RATE = Math.PI / 2.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate3() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate4() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate5() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate6() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate7() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this, 
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate8() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate9() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate10() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate11() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(position, calibrator.getEcefPosition());
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate12() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate13() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate14() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate15() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate16() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate17() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
                false, false, initialBias, initialMg, initialGg, 
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate18() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate19() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate20() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertArrayEquals(initialBias, calibrator.getInitialBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate21() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate22() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate23() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate24() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate25() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate26() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate27() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertTrue(calibrator.getNedPosition().equals(position, ABSOLUTE_ERROR));
        assertEquals(ROTATION_RATE, calibrator.getTurntableRotationRate(), 0.0);
        assertEquals(TIME_INTERVAL, calibrator.getTimeInterval(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getInitialBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate28() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate29() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate30() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate31() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate32() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate33() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate34() {
        final var qualityScores = new double[10];

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate35() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate36() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate37() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate38() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate39() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate40() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate41() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate42() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate43() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, true, true, initialBias,
                initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate44() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, true, true, initialBias,
                initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate45() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate46() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate47() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate48() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate49() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate50() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate51() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate52() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate53() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate54() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate55() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate56() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate57() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate58() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate59() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate60() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, true, true, initialBias,
                initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate61() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, true, true, initialBias,
                initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate62() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate63() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate64() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate65() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate66() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate67() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreate68() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustTurntableGyroscopeCalibrator.create(qualityScores, position, ROTATION_RATE,
                TIME_INTERVAL, measurements, false, false, initialBias,
                initialMg, initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(LMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(MSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROSACRobustTurntableGyroscopeCalibrator.class, calibrator);
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
        assertInstanceOf(PROMedSRobustTurntableGyroscopeCalibrator.class, calibrator);
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
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustTurntableGyroscopeCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod3() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this);

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
    void testCreateWithDefaultMethod4() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod5() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this);

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
    void testCreateWithDefaultMethod6() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod7() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod8() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod9() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod10() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                true, true, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod11() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL, measurements,
                true, true, initialBias, initialMg, initialGg, this);

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
    void testCreateWithDefaultMethod12() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

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
    void testCreateWithDefaultMethod13() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod14() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg,
                initialGg);

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
    void testCreateWithDefaultMethod15() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this);

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
    void testCreateWithDefaultMethod16() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod17() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod18() throws WrongSizeException {
        final var position = new ECEFPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod19() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod20() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this);

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
    void testCreateWithDefaultMethod21() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod22() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, this);

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
    void testCreateWithDefaultMethod23() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod24() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod25() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod26() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod27() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg);

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
    void testCreateWithDefaultMethod28() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, true, true, initialBias, initialMg, initialGg,
                this);

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
    void testCreateWithDefaultMethod29() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod30() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this);

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
    void testCreateWithDefaultMethod31() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg,
                initialGg);

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
    void testCreateWithDefaultMethod32() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                this);

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
    void testCreateWithDefaultMethod33() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);

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
    void testCreateWithDefaultMethod34() throws WrongSizeException {
        final var position = new NEDPosition();
        final var measurements = Collections.<StandardDeviationBodyKinematics>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustTurntableGyroscopeCalibrator.create(position, ROTATION_RATE, TIME_INTERVAL,
                measurements, false, false, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this);

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
