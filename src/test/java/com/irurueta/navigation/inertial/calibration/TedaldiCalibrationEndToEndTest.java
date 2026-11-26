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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.geometry.Quaternion;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end test of the IMU calibration pipeline following the procedure described in
 * Tedaldi et al., "A robust and easy to implement method for IMU calibration without external equipments",
 * ICRA 2014: an initial static period followed by a set of static positions separated by slow and fast rotations.
 * <p>
 * Synthetic data is generated with a known error model (upper triangular accelerometer scale factors and cross
 * couplings, general gyroscope scale factors and cross couplings, biases and white noise), processed by the
 * measurements generator and calibrated with the default norm-based accelerometer calibrator and the default easy
 * gyroscope calibrator. All random data is seeded so that the test is fully deterministic.
 */
class TedaldiCalibrationEndToEndTest {

    private static final double TIME_INTERVAL = 0.01;

    private static final double GRAVITY = 9.80665;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;

    // 100 micro-g / sqrt(Hz) accelerometer noise and 0.01 deg/s/sqrt(Hz) gyroscope noise (consumer grade MEMS)
    private static final double ACCEL_NOISE_ROOT_PSD = 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    private static final double GYRO_NOISE_ROOT_PSD = Math.toRadians(0.01);

    private static final int INITIAL_STATIC_SAMPLES = 500;
    private static final int STATIC_SAMPLES = 400;
    private static final int SLOW_ROTATION_SAMPLES = 200;
    private static final int FAST_ROTATION_SAMPLES = 100;
    private static final double SLOW_ROTATION_DEGREES = 45.0;
    private static final double FAST_ROTATION_DEGREES = 120.0;

    private static final int MIN_POSITIONS = 36;
    private static final int MAX_POSITIONS = 50;

    // Tolerances. Noise (100 ug/sqrt(Hz) accelerometer, 0.01 deg/s/sqrt(Hz) gyroscope at 100 Hz) averaged over
    // static intervals of ~200 samples gives a specific force standard deviation of ~7e-4 m/s^2 (7e-5 relative
    // to g). Over 17 different seeds the worst observed absolute errors were: Ma 1.0e-4, ba 4.5e-4 m/s^2,
    // Mg 2.2e-4, bg 7e-5 rad/s and 7e-4 for the integrated rotation matrix. Tolerances keep a margin of about
    // 3x over those worst cases, while still being 1-2 orders of magnitude below the true parameter values
    // (1e-2 for scale factors and cross couplings), so the test fails if any model or detection error appears.
    private static final double MA_ABSOLUTE_ERROR = 3e-4;
    private static final double BA_ABSOLUTE_ERROR = 1.5e-3;
    private static final double MG_ABSOLUTE_ERROR = 6e-4;
    private static final double BG_ABSOLUTE_ERROR = 2e-4;

    // integrated rotation of the slow-start sequence: limited by the integrated gyroscope noise and by the few
    // samples at the interval boundaries
    private static final double ROTATION_ABSOLUTE_ERROR = 3e-3;

    private static final long[] SEEDS = {2026L, 7L, 12345L};

    @Test
    void testCalibrationWithSyntheticData() throws Exception {
        for (final var seed : SEEDS) {
            final var result = run(seed);
            result.assertCalibration();
        }
    }

    @Test
    void testIntegratedRotationOfSlowStartSequence() throws Exception {
        for (final var seed : SEEDS) {
            final var result = run(seed);
            result.assertFirstSequenceRotation();
        }
    }

    @Test
    void testPipelineIsDeterministic() throws Exception {
        final var result1 = run(SEEDS[0]);
        final var result2 = run(SEEDS[0]);

        assertEquals(result1.accelerometerMeasurements.size(), result2.accelerometerMeasurements.size());
        assertEquals(result1.sequences.size(), result2.sequences.size());
        assertTrue(result1.estimatedMa.equals(result2.estimatedMa, 0.0));
        assertTrue(result1.estimatedMg.equals(result2.estimatedMg, 0.0));
    }

    private static Result run(final long seed) throws Exception {
        final var random = new Random(seed);
        final var scenario = new Scenario(random);

        final var collector = new Collector();
        final var generator = new AccelerometerAndGyroscopeMeasurementsGenerator(collector);
        generator.setTimeInterval(TIME_INTERVAL);
        generator.setInitialStaticSamples(INITIAL_STATIC_SAMPLES);

        for (final var sample : scenario.samples) {
            assertTrue(generator.process(sample));
        }

        assertFalse(collector.error);
        assertNotNull(generator.getStatus());
        assertTrue(generator.getStatus() != TriadStaticIntervalDetector.Status.FAILED);

        final var accelerometerMeasurements = collector.accelerometerMeasurements;
        final var sequences = collector.sequences;

        // accelerometer: default is upper triangular (common Z axis)
        final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(GRAVITY,
                accelerometerMeasurements);
        assertTrue(accelerometerCalibrator.isCommonAxisUsed());
        accelerometerCalibrator.calibrate();

        final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
        final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

        // gyroscope: default is general Mg without G-dependent cross biases. Initial bias is the average
        // gyroscope measurement during the initial static period.
        final var gyroscopeCalibrator = new EasyGyroscopeCalibrator();
        assertFalse(gyroscopeCalibrator.isCommonAxisUsed());
        assertFalse(gyroscopeCalibrator.isGDependentCrossBiasesEstimated());
        gyroscopeCalibrator.setSequences(sequences);
        gyroscopeCalibrator.setInitialBias(generator.getInitialAvgAngularSpeedTriad().getValuesAsMatrix());
        gyroscopeCalibrator.setAccelerometerBias(estimatedBa);
        gyroscopeCalibrator.setAccelerometerMa(estimatedMa);
        gyroscopeCalibrator.calibrate();

        return new Result(scenario, accelerometerMeasurements, sequences, estimatedBa, estimatedMa,
                gyroscopeCalibrator.getEstimatedBiasesAsMatrix(), gyroscopeCalibrator.getEstimatedMg());
    }

    private static Matrix matrixOf(final double... values) throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(values, false);
        return result;
    }

    private static Matrix trueBa() {
        return Matrix.newFromArray(new double[]{0.04, -0.06, 0.03});
    }

    private static Matrix trueBg() {
        return Matrix.newFromArray(new double[]{0.004, -0.006, 0.003});
    }

    private static Matrix trueMa() throws WrongSizeException {
        return matrixOf(
                0.010, -0.005, 0.003,
                0.0, -0.008, 0.004,
                0.0, 0.0, 0.006);
    }

    private static Matrix trueMg() throws WrongSizeException {
        return matrixOf(
                0.010, -0.004, 0.003,
                0.002, -0.008, -0.005,
                -0.003, 0.004, 0.006);
    }

    /**
     * Synthetic motion: contains the timestamped measured kinematics and the true rotation of the first
     * rotation (slow start).
     */
    private static class Scenario {

        private final List<TimedBodyKinematics> samples = new ArrayList<>();

        // true rotation (body frame increment) of the first rotation, row-major 3x3
        private double[] firstRotation;

        // sample index where the first rotation starts
        private int firstRotationStart;

        // sample index (exclusive) where the first rotation ends
        private int firstRotationEnd;

        Scenario(final Random random) throws WrongSizeException {
            final var errors = new IMUErrors(trueBa(), trueBg(), trueMa(), trueMg(), new Matrix(3, 3),
                    ACCEL_NOISE_ROOT_PSD, GYRO_NOISE_ROOT_PSD, 0.0, 0.0);

            // orientation: body to local frame rotation matrix, row-major
            final var rotation = new double[]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

            final var trueKinematics = new BodyKinematics();
            final var measured = new BodyKinematics();

            var index = 0;

            // initial static period plus first static position
            index = addStatic(random, errors, rotation, trueKinematics, measured, index,
                    INITIAL_STATIC_SAMPLES + STATIC_SAMPLES);

            final var positions = MIN_POSITIONS + random.nextInt(MAX_POSITIONS - MIN_POSITIONS + 1);
            for (var i = 0; i < positions; i++) {
                // first rotation is slow, then alternate slow and fast ones
                final var slow = i % 2 == 0;
                final var numSamples = slow ? SLOW_ROTATION_SAMPLES : FAST_ROTATION_SAMPLES;
                final var angle = Math.toRadians(slow ? SLOW_ROTATION_DEGREES : FAST_ROTATION_DEGREES);
                final var axis = randomAxis(random);

                final var start = index;
                index = addRotation(random, errors, rotation, trueKinematics, measured, index, numSamples, angle,
                        axis);

                if (i == 0) {
                    firstRotation = rodrigues(axis, angle);
                    firstRotationStart = start;
                    firstRotationEnd = index;
                }

                index = addStatic(random, errors, rotation, trueKinematics, measured, index, STATIC_SAMPLES);
            }

            // a last rotation is required to finish the last static position
            addRotation(random, errors, rotation, trueKinematics, measured, index, SLOW_ROTATION_SAMPLES,
                    Math.toRadians(SLOW_ROTATION_DEGREES), randomAxis(random));
        }

        private int addStatic(
                final Random random, final IMUErrors errors, final double[] rotation,
                final BodyKinematics trueKinematics, final BodyKinematics measured, final int start,
                final int numSamples) {
            setTrueKinematics(rotation, 0.0, 0.0, 0.0, trueKinematics);
            var index = start;
            for (var i = 0; i < numSamples; i++) {
                addSample(random, errors, trueKinematics, measured, index);
                index++;
            }
            return index;
        }

        private int addRotation(
                final Random random, final IMUErrors errors, final double[] rotation,
                final BodyKinematics trueKinematics, final BodyKinematics measured, final int start,
                final int numSamples, final double angle, final double[] axis) {
            // raised cosine angular rate profile (smooth start and end) whose integral is the requested angle
            final var weights = new double[numSamples];
            var sum = 0.0;
            for (var i = 0; i < numSamples; i++) {
                weights[i] = 1.0 - Math.cos(2.0 * Math.PI * (i + 0.5) / numSamples);
                sum += weights[i];
            }

            var index = start;
            for (var i = 0; i < numSamples; i++) {
                final var rate = angle * weights[i] / (sum * TIME_INTERVAL);
                setTrueKinematics(rotation, axis[0] * rate, axis[1] * rate, axis[2] * rate, trueKinematics);
                addSample(random, errors, trueKinematics, measured, index);
                index++;

                // update orientation (body frame increment about a fixed body axis)
                final var increment = rodrigues(axis, rate * TIME_INTERVAL);
                multiplyInPlace(rotation, increment);
            }
            return index;
        }

        private void addSample(
                final Random random, final IMUErrors errors, final BodyKinematics trueKinematics,
                final BodyKinematics measured, final int index) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL, trueKinematics, errors, random, measured);
            samples.add(new TimedBodyKinematics(new BodyKinematics(measured), index * TIME_INTERVAL));
        }

        // specific force in body frame is the opposite of gravity expressed in body frame
        private static void setTrueKinematics(
                final double[] rotation, final double wx, final double wy, final double wz,
                final BodyKinematics result) {
            // R^T * (0, 0, -g) where the third column of R^T is the third row of R
            result.setSpecificForceCoordinates(-GRAVITY * rotation[6], -GRAVITY * rotation[7],
                    -GRAVITY * rotation[8]);
            result.setAngularRateCoordinates(wx, wy, wz);
        }

        private static double[] randomAxis(final Random random) {
            final var axis = new double[]{random.nextGaussian(), random.nextGaussian(), random.nextGaussian()};
            final var norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            return new double[]{axis[0] / norm, axis[1] / norm, axis[2] / norm};
        }

        // Rodrigues' formula, row-major 3x3 rotation matrix of the given angle about the given unit axis
        private static double[] rodrigues(final double[] axis, final double angle) {
            final var c = Math.cos(angle);
            final var s = Math.sin(angle);
            final var t = 1.0 - c;
            final var x = axis[0];
            final var y = axis[1];
            final var z = axis[2];
            return new double[]{
                    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                    t * x * z - s * y, t * y * z + s * x, t * z * z + c};
        }

        // a = a * b
        private static void multiplyInPlace(final double[] a, final double[] b) {
            final var result = new double[9];
            for (var i = 0; i < 3; i++) {
                for (var j = 0; j < 3; j++) {
                    var value = 0.0;
                    for (var k = 0; k < 3; k++) {
                        value += a[i * 3 + k] * b[k * 3 + j];
                    }
                    result[i * 3 + j] = value;
                }
            }
            System.arraycopy(result, 0, a, 0, 9);
        }
    }

    private static class Collector implements AccelerometerAndGyroscopeMeasurementsGeneratorListener {

        private final List<StandardDeviationBodyKinematics> accelerometerMeasurements = new ArrayList<>();

        private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                new ArrayList<>();

        private boolean error;

        @Override
        public void onInitializationStarted(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }

        @Override
        public void onInitializationCompleted(
                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                final double accelerometerBaseNoiseLevel) {
            // not needed
        }

        @Override
        public void onError(
                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                final TriadStaticIntervalDetector.ErrorReason reason) {
            error = true;
        }

        @Override
        public void onStaticIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }

        @Override
        public void onDynamicIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }

        @Override
        public void onStaticIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }

        @Override
        public void onDynamicIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }

        @Override
        public void onGeneratedAccelerometerMeasurement(
                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                final StandardDeviationBodyKinematics measurement) {
            accelerometerMeasurements.add(measurement);
        }

        @Override
        public void onGeneratedGyroscopeMeasurement(
                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
            sequences.add(measurement);
        }

        @Override
        public void onReset(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
            // not needed
        }
    }

    private record Result(Scenario scenario,
                          List<StandardDeviationBodyKinematics> accelerometerMeasurements,
                          List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
                          Matrix estimatedBa, Matrix estimatedMa, Matrix estimatedBg, Matrix estimatedMg) {

        void assertCalibration() throws WrongSizeException {
            // enough static positions and rotations were detected
            assertTrue(accelerometerMeasurements.size() >= MIN_POSITIONS);
            assertTrue(sequences.size() >= MIN_POSITIONS);

            assertMatrixEquals(trueMa(), estimatedMa, MA_ABSOLUTE_ERROR, "Ma");
            assertMatrixEquals(trueBa(), estimatedBa, BA_ABSOLUTE_ERROR, "ba");
            assertMatrixEquals(trueMg(), estimatedMg, MG_ABSOLUTE_ERROR, "Mg");
            assertMatrixEquals(trueBg(), estimatedBg, BG_ABSOLUTE_ERROR, "bg");
        }

        void assertFirstSequenceRotation() throws Exception {
            assertFalse(sequences.isEmpty());

            // the first sequence is the one generated by the slow rotation that follows the initial static period
            final var sequence = sequences.get(0);
            final var items = sequence.getSortedItems();

            // the detected interval must contain the whole true rotation (half window delay is compensated)
            final var firstTimestamp = items.get(0).getTimestampSeconds();
            final var lastTimestamp = items.get(items.size() - 1).getTimestampSeconds();
            assertTrue(firstTimestamp <= scenario.firstRotationStart * TIME_INTERVAL);
            assertTrue(lastTimestamp >= (scenario.firstRotationEnd - 1) * TIME_INTERVAL);

            // calibrate the angular rates with the true parameters and integrate them
            final var mgInv = com.irurueta.algebra.Utils.inverse(trueMg().addAndReturnNew(Matrix.identity(3, 3)));
            final var bg = trueBg();
            final var corrected = new ArrayList<StandardDeviationTimedBodyKinematics>();
            for (final var item : items) {
                final var kinematics = item.getKinematics();
                final var w = Matrix.newFromArray(new double[]{kinematics.getAngularRateX(),
                        kinematics.getAngularRateY(), kinematics.getAngularRateZ()});
                w.subtract(bg);
                final var calibrated = mgInv.multiplyAndReturnNew(w);
                final var calibratedKinematics = new BodyKinematics(0.0, 0.0, 0.0, calibrated.getElementAtIndex(0),
                        calibrated.getElementAtIndex(1), calibrated.getElementAtIndex(2));
                corrected.add(new StandardDeviationTimedBodyKinematics(calibratedKinematics,
                        item.getTimestampSeconds(), 0.0, 0.0));
            }
            final var correctedSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
            correctedSequence.setItems(corrected);

            final var integrated = new Quaternion();
            QuaternionIntegrator.integrateGyroSequence(correctedSequence, QuaternionStepIntegratorType.RUNGE_KUTTA,
                    integrated);

            final var integratedMatrix = integrated.asInhomogeneousMatrix();
            final var expected = matrixOf(scenario.firstRotation);

            assertMatrixEquals(expected, integratedMatrix, ROTATION_ABSOLUTE_ERROR, "integrated rotation");
        }

        private static void assertMatrixEquals(
                final Matrix expected, final Matrix actual, final double tolerance, final String name) {
            assertEquals(expected.getRows(), actual.getRows(), name);
            assertEquals(expected.getColumns(), actual.getColumns(), name);
            for (var i = 0; i < expected.getRows(); i++) {
                for (var j = 0; j < expected.getColumns(); j++) {
                    assertEquals(expected.getElementAt(i, j), actual.getElementAt(i, j), tolerance,
                            name + "[" + i + "," + j + "]");
                }
            }
        }
    }
}
