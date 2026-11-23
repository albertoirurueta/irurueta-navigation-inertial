/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests how generators handle static intervals using deterministic synthetic data (N4, N6 and N9).
 */
class GeneratorsStaticIntervalTest {

    private static final double DT = 0.02;
    private static final int WINDOW = 21;
    private static final int DELAY = WINDOW / 2;
    private static final int INITIAL = 200;
    private static final int MIN_STATIC = 60;
    private static final double ACC_STD = 0.01;
    private static final double GYRO_STD = 1e-3;
    private static final double DYNAMIC_ACC_STD = 1.0;
    private static final double GYRO_RATE = 0.5;
    private static final double DRIFT = 0.1;

    private static final double[] A = {0.0, 0.0, 9.8};
    private static final double[] B = {0.0, 9.8, 0.0};

    private final Random random = new Random(1234);
    private int index;

    @Test
    void shortStaticIntervalJoinsNeighbouringDynamicSequences() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        // initialization, dynamic, short static (invalid), dynamic, long static (valid), dynamic
        feedStatic(generator, A, INITIAL, 0.0);
        final var dynamic1Start = index;
        feedDynamic(generator, 80);
        feedStatic(generator, A, MIN_STATIC / 2, 0.0);
        feedDynamic(generator, 80);
        feedStatic(generator, B, 150, DRIFT);
        assertTrue(sequences.isEmpty());
        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        final var sequence = sequences.get(0);
        // dynamic interval is extended half a window after it ends by the centred window (it cannot be extended
        // before the end of initialization) and the short static interval is part of the joined sequence
        final var expected = 80 + MIN_STATIC / 2 + 80 + DELAY;
        assertEquals(expected, sequence.getItemsCount(), 6);
        assertEquals(A[2], sequence.getBeforeMeanFz(), 0.01);
        assertEquals(B[1] + DRIFT / 2.0, sequence.getAfterMeanFy(), 0.01);
        assertEquals((dynamic1Start + 1) * DT, sequence.getSortedItems().get(0).getTimestampSeconds(), 2 * DT);
    }

    @Test
    void afterVectorIsAverageOfWholeFollowingStaticIntervalAndEmittedWhenItEnds() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        feedStatic(generator, A, INITIAL, 0.0);
        feedStatic(generator, A, 100, 0.0);
        feedDynamic(generator, 80);
        feedStatic(generator, B, 200, DRIFT);

        // static interval has not ended yet
        assertTrue(sequences.isEmpty());

        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        final var sequence = sequences.get(0);
        // average of the whole interval (drifts from B to B + DRIFT), not the last window
        assertEquals(B[1] + DRIFT / 2.0, sequence.getAfterMeanFy(), 0.01);
        assertEquals(A[2], sequence.getBeforeMeanFz(), 0.01);
    }

    @Test
    void motionRightAfterInitializationUsesInitializationAverageAsBeforeVector() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        feedStatic(generator, A, INITIAL, 0.0);
        feedDynamic(generator, 100);
        feedStatic(generator, B, 150, 0.0);
        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        final var sequence = sequences.get(0);
        assertEquals(0.0, sequence.getBeforeMeanFx(), 0.01);
        assertEquals(0.0, sequence.getBeforeMeanFy(), 0.01);
        assertEquals(A[2], sequence.getBeforeMeanFz(), 0.01);
        assertEquals(B[1], sequence.getAfterMeanFy(), 0.01);
        assertEquals(100 + DELAY, sequence.getItemsCount(), 4);
    }

    @Test
    void sequenceItemsAreAlignedWithDetectedDynamicInterval() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        feedStatic(generator, A, INITIAL, 0.0);
        feedStatic(generator, A, 100, 0.0);
        final var dynamicStart = index;
        feedDynamic(generator, 80);
        feedStatic(generator, B, 150, 0.0);
        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        final var sequence = sequences.get(0);
        // centred window classifies as dynamic the samples that are within half a window of the motion
        assertEquals((dynamicStart + 1 - DELAY) * DT, sequence.getSortedItems().get(0).getTimestampSeconds(),
                2 * DT);
        assertEquals(80 + 2 * DELAY, sequence.getItemsCount(), 4);
    }

    @Test
    void sequenceWithoutPrecedingValidStaticIntervalIsDiscarded() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        feedStatic(generator, A, INITIAL, 0.0);
        // too long dynamic interval makes gravity reference be lost
        generator.setMaxDynamicSamples(50);
        feedDynamic(generator, 100);
        feedStatic(generator, B, 150, 0.0);
        feedDynamic(generator, 60);

        // dynamic interval was skipped, so there is no reference before it
        assertTrue(sequences.isEmpty());
    }

    @Test
    void gravityDirectionStandardDeviationCombinesStaticMeansAndGyroscopeNoise() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var generator = createGenerator(sequences);

        feedStatic(generator, A, INITIAL, 0.0);
        feedDynamic(generator, 100);
        feedStatic(generator, B, 150, 0.0);
        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        final var sequence = sequences.get(0);
        final var items = sequence.getSortedItems();
        final var duration = items.get(items.size() - 1).getTimestampSeconds() - items.get(0).getTimestampSeconds();
        final var accStd = generator.getAccelerometerBaseNoiseLevelRootPsd() / Math.sqrt(DT);
        final var rootPsd = generator.getGyroscopeBaseNoiseLevelRootPsd();
        final var g = 9.8;
        final var nAfter = 150 - 2 * DELAY;
        final var expected = Math.sqrt(accStd * accStd / (g * g) / INITIAL + accStd * accStd / (g * g) / nAfter
                + rootPsd * rootPsd * duration);
        assertTrue(expected > 0.0);
        assertEquals(expected, sequence.getGravityDirectionStandardDeviation(), 0.03 * expected);
    }

    @Test
    void accelerometerGeneratorReportsStandardDeviationOfTheAverage() throws LockedException {
        final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
        final var generator = new AccelerometerMeasurementsGenerator(new AccelerometerListener(measurements));
        configure(generator);

        feedAccelerometer(generator, A, INITIAL);
        feedAccelerometer(generator, A, 200);
        feedAccelerometerDynamic(generator, 60);
        feedAccelerometer(generator, B, 200);
        feedAccelerometerDynamic(generator, 60);

        assertEquals(2, measurements.size());
        // first static interval contains the initialization samples (static)
        final var n1 = INITIAL + 200 - DELAY;
        assertEquals(ACC_STD / Math.sqrt(n1), measurements.get(0).getSpecificForceStandardDeviation(),
                0.3 * ACC_STD / Math.sqrt(n1));
        final var n2 = 200 - 2 * DELAY;
        assertEquals(ACC_STD / Math.sqrt(n2), measurements.get(1).getSpecificForceStandardDeviation(),
                0.3 * ACC_STD / Math.sqrt(n2));
    }

    @Test
    void combinedGeneratorsGenerateSequencesAndAverageStandardDeviations() throws LockedException {
        final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        final var measurements = new ArrayList<StandardDeviationBodyKinematics>();
        final var generator = new AccelerometerAndGyroscopeMeasurementsGenerator(
                new AccelerometerAndGyroscopeMeasurementsGeneratorListener() {
                    @Override
                    public void onInitializationStarted(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }

                    @Override
                    public void onInitializationCompleted(
                            final AccelerometerAndGyroscopeMeasurementsGenerator g, final double baseNoiseLevel) {
                        // no action needed
                    }

                    @Override
                    public void onError(final AccelerometerAndGyroscopeMeasurementsGenerator g,
                                        final com.irurueta.navigation.inertial.calibration.intervals
                                                .TriadStaticIntervalDetector.ErrorReason reason) {
                        // no action needed
                    }

                    @Override
                    public void onStaticIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }

                    @Override
                    public void onDynamicIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }

                    @Override
                    public void onStaticIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }

                    @Override
                    public void onDynamicIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }

                    @Override
                    public void onGeneratedAccelerometerMeasurement(
                            final AccelerometerAndGyroscopeMeasurementsGenerator g,
                            final StandardDeviationBodyKinematics measurement) {
                        measurements.add(measurement);
                    }

                    @Override
                    public void onGeneratedGyroscopeMeasurement(
                            final AccelerometerAndGyroscopeMeasurementsGenerator g,
                            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                        sequences.add(sequence);
                    }

                    @Override
                    public void onReset(final AccelerometerAndGyroscopeMeasurementsGenerator g) {
                        // no action needed
                    }
                });
        generator.setTimeInterval(DT);
        generator.setWindowSize(WINDOW);
        generator.setInitialStaticSamples(INITIAL);
        generator.setMinStaticSamples(MIN_STATIC);

        feedStatic(generator, A, INITIAL, 0.0);
        feedDynamic(generator, 100);
        feedStatic(generator, B, 200, 0.0);
        feedDynamic(generator, 60);

        assertEquals(1, sequences.size());
        assertEquals(A[2], sequences.get(0).getBeforeMeanFz(), 0.01);
        assertEquals(B[1], sequences.get(0).getAfterMeanFy(), 0.01);
        assertTrue(sequences.get(0).getGravityDirectionStandardDeviation() > 0.0);
        assertEquals(1, measurements.size());
        final var n = 200 - 2 * DELAY;
        assertEquals(ACC_STD / Math.sqrt(n), measurements.get(0).getSpecificForceStandardDeviation(),
                0.3 * ACC_STD / Math.sqrt(n));
    }

    private GyroscopeMeasurementsGenerator createGenerator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences)
            throws LockedException {
        final var generator = new GyroscopeMeasurementsGenerator(new GyroscopeListener(sequences));
        configure(generator);
        return generator;
    }

    private static void configure(final MeasurementsGenerator<?, ?, ?, ?> generator) throws LockedException {
        generator.setTimeInterval(DT);
        generator.setWindowSize(WINDOW);
        generator.setInitialStaticSamples(INITIAL);
        generator.setMinStaticSamples(MIN_STATIC);
    }

    private TimedBodyKinematics sample(final double[] f, final double drift, final int n, final int count,
                                       final boolean dynamic) {
        final var accStd = dynamic ? DYNAMIC_ACC_STD : ACC_STD;
        final var kinematics = new BodyKinematics();
        final var fy = f[1] + (count > 1 ? drift * n / (count - 1) : 0.0);
        kinematics.setSpecificForceCoordinates(
                f[0] + accStd * random.nextGaussian(),
                fy + accStd * random.nextGaussian(),
                f[2] + accStd * random.nextGaussian());
        final var rate = dynamic ? GYRO_RATE : 0.0;
        kinematics.setAngularRateCoordinates(
                GYRO_STD * random.nextGaussian(),
                GYRO_STD * random.nextGaussian(),
                rate + GYRO_STD * random.nextGaussian());
        index++;
        return new TimedBodyKinematics(kinematics, index * DT);
    }

    private void feedStatic(final GyroscopeMeasurementsGenerator generator, final double[] f, final int count,
                            final double drift) throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(f, drift, i, count, false)));
        }
    }

    private void feedDynamic(final GyroscopeMeasurementsGenerator generator, final int count)
            throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(A, 0.0, i, count, true)));
        }
    }

    private void feedStatic(final AccelerometerAndGyroscopeMeasurementsGenerator generator, final double[] f,
                            final int count, final double drift) throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(f, drift, i, count, false)));
        }
    }

    private void feedDynamic(final AccelerometerAndGyroscopeMeasurementsGenerator generator, final int count)
            throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(A, 0.0, i, count, true)));
        }
    }

    private void feedAccelerometer(final AccelerometerMeasurementsGenerator generator, final double[] f,
                                   final int count) throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(f, 0.0, i, count, false).getKinematics()));
        }
    }

    private void feedAccelerometerDynamic(final AccelerometerMeasurementsGenerator generator, final int count)
            throws LockedException {
        for (var i = 0; i < count; i++) {
            assertTrue(generator.process(sample(A, 0.0, i, count, true).getKinematics()));
        }
    }

    private static class GyroscopeListener implements GyroscopeMeasurementsGeneratorListener {

        private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences;

        GyroscopeListener(final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences) {
            this.sequences = sequences;
        }

        @Override
        public void onInitializationStarted(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onInitializationCompleted(final GyroscopeMeasurementsGenerator generator,
                                              final double baseNoiseLevel) {
            // no action needed
        }

        @Override
        public void onError(final GyroscopeMeasurementsGenerator generator,
                            final com.irurueta.navigation.inertial.calibration.intervals
                                    .TriadStaticIntervalDetector.ErrorReason reason) {
            // no action needed
        }

        @Override
        public void onStaticIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onDynamicIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onStaticIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onDynamicIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onGeneratedMeasurement(
                final GyroscopeMeasurementsGenerator generator,
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
            sequences.add(measurement);
        }

        @Override
        public void onReset(final GyroscopeMeasurementsGenerator generator) {
            // no action needed
        }
    }

    private static class AccelerometerListener implements AccelerometerMeasurementsGeneratorListener {

        private final List<StandardDeviationBodyKinematics> measurements;

        AccelerometerListener(final List<StandardDeviationBodyKinematics> measurements) {
            this.measurements = measurements;
        }

        @Override
        public void onInitializationStarted(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onInitializationCompleted(final AccelerometerMeasurementsGenerator generator,
                                              final double baseNoiseLevel) {
            // no action needed
        }

        @Override
        public void onError(final AccelerometerMeasurementsGenerator generator,
                            final com.irurueta.navigation.inertial.calibration.intervals
                                    .TriadStaticIntervalDetector.ErrorReason reason) {
            // no action needed
        }

        @Override
        public void onStaticIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onDynamicIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onStaticIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onDynamicIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }

        @Override
        public void onGeneratedMeasurement(final AccelerometerMeasurementsGenerator generator,
                                           final StandardDeviationBodyKinematics measurement) {
            measurements.add(measurement);
        }

        @Override
        public void onReset(final AccelerometerMeasurementsGenerator generator) {
            // no action needed
        }
    }
}
