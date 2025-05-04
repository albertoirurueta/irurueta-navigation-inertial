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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class RobustKnownBiasEasyGyroscopeCalibratorTest implements
        RobustKnownBiasEasyGyroscopeCalibratorListener {

    @Test
    void testCreate1() {
        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
    }

    @Test
    void testCreate2() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate3() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate4() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate5() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate6() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreate7() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate8() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreate9() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate10() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate11() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate12() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreate13() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate14() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(),0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreate15() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate16() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(), accelerometerBias);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreate17() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreate18() {
        final var qualityScores = new double[10];

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate19() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate20() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate21() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate22() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate23() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate24() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate25() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate26() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate27() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate28() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate29() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate30() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate31() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(calibrator.getAccelerometerMa(), accelerometerMa);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate32() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate33() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreate34() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        // RANSAC
        var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(calibrator.getInitialGg(), initialGg);
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(), accelerometerBias);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());

        // PROMedS
        calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(qualityScores, sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.class, calibrator);
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
        assertSame(qualityScores, calibrator.getQualityScores());
    }

    @Test
    void testCreateWithDefaultMethod1() {
        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create();

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
    }

    @Test
    void testCreateWithDefaultMethod2() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreateWithDefaultMethod3() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod4() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreateWithDefaultMethod5() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod6() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreateWithDefaultMethod7() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod8() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreateWithDefaultMethod9() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod10() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreateWithDefaultMethod11() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod12() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
    }

    @Test
    void testCreateWithDefaultMethod13() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod14() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreateWithDefaultMethod15() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new double[3];
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new double[3];
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertArrayEquals(initialBias, calibrator.getBias(), 0.0);
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertArrayEquals(accelerometerBias, calibrator.getAccelerometerBias(), 0.0);
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testCreateWithDefaultMethod16() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
    }

    @Test
    void testCreateWithDefaultMethod17() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        final var initialBias = new Matrix(3, 1);
        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        final var accelerometerBias = new Matrix(3, 1);
        final var accelerometerMa = new Matrix(3, 3);

        final var calibrator = RobustKnownBiasEasyGyroscopeCalibrator.create(sequences, false,
                false, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                this);

        // check
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertSame(sequences, calibrator.getSequences());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertEquals(initialBias, calibrator.getBiasAsMatrix());
        assertEquals(initialMg, calibrator.getInitialMg());
        assertEquals(initialGg, calibrator.getInitialGg());
        assertEquals(accelerometerBias, calibrator.getAccelerometerBiasAsMatrix());
        assertEquals(accelerometerMa, calibrator.getAccelerometerMa());
        assertSame(this, calibrator.getListener());
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
        // no action needed
    }

    @Override
    public void onCalibrateNextIteration(final RobustKnownBiasEasyGyroscopeCalibrator calibrator, final int iteration) {
        // no action needed
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasEasyGyroscopeCalibrator calibrator, final float progress) {
        // no action needed
    }
}
