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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.GyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownFrameGyroscopeLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class IMUErrorsCreatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;


    @Test
    void testConstructor1() {
        final var creator = new IMUErrorsCreator();

        // check default values
        assertNull(creator.getAccelerometerCalibrationSource());
        assertNull(creator.getGyroscopeCalibrationSource());
        assertNull(creator.getAccelerometerNoiseRootPsdSource());
        assertNull(creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    void testConstructor2() {
        final var accelerometerCalibrator = new KnownFrameAccelerometerLinearLeastSquaresCalibrator();
        final var gyroscopeCalibrator = new KnownFrameGyroscopeLinearLeastSquaresCalibrator();
        final var accelerometerGenerator = new AccelerometerMeasurementsGenerator();
        final var gyroscopeGenerator = new GyroscopeMeasurementsGenerator();

        final var creator = new IMUErrorsCreator(accelerometerCalibrator, gyroscopeCalibrator, accelerometerGenerator,
                gyroscopeGenerator);

        // check default values
        assertSame(accelerometerCalibrator, creator.getAccelerometerCalibrationSource());
        assertSame(gyroscopeCalibrator, creator.getGyroscopeCalibrationSource());
        assertSame(accelerometerGenerator, creator.getAccelerometerNoiseRootPsdSource());
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    void testConstructor3() {
        final var accelerometerCalibrator = new KnownFrameAccelerometerLinearLeastSquaresCalibrator();
        final var gyroscopeCalibrator = new KnownFrameGyroscopeLinearLeastSquaresCalibrator();
        final var accelerometerGenerator = new AccelerometerMeasurementsGenerator();
        final var gyroscopeGenerator = new GyroscopeMeasurementsGenerator();

        final var random = new Random();
        final var accelerometerQuantizationLevel = random.nextDouble();
        final var gyroscopeQuantizationLevel = random.nextDouble();

        final var creator = new IMUErrorsCreator(accelerometerCalibrator, gyroscopeCalibrator, accelerometerGenerator,
                gyroscopeGenerator, accelerometerQuantizationLevel, gyroscopeQuantizationLevel);

        // check default values
        assertSame(accelerometerCalibrator, creator.getAccelerometerCalibrationSource());
        assertSame(gyroscopeCalibrator, creator.getGyroscopeCalibrationSource());
        assertSame(accelerometerGenerator, creator.getAccelerometerNoiseRootPsdSource());
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
        assertEquals(accelerometerQuantizationLevel, creator.getAccelerometerQuantizationLevel(), 0.0);
        assertEquals(gyroscopeQuantizationLevel, creator.getGyroQuantizationLevel(), 0.0);
        assertFalse(creator.isReady());
    }

    @Test
    void testGetSetAccelerometerCalibrationSource() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getAccelerometerCalibrationSource());

        // set new value
        final var accelerometerCalibrator = new KnownFrameAccelerometerLinearLeastSquaresCalibrator();

        creator.setAccelerometerCalibrationSource(accelerometerCalibrator);

        // check
        assertSame(accelerometerCalibrator, creator.getAccelerometerCalibrationSource());
    }

    @Test
    void testGetSetGyroscopeCalibrationSource() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getGyroscopeCalibrationSource());

        // set new value
        final var gyroscopeCalibrator = new KnownFrameGyroscopeLinearLeastSquaresCalibrator();

        creator.setGyroscopeCalibrationSource(gyroscopeCalibrator);

        // check
        assertSame(gyroscopeCalibrator, creator.getGyroscopeCalibrationSource());
    }

    @Test
    void testGetSetAccelerometerNoiseRootPsdSource() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getAccelerometerNoiseRootPsdSource());

        // set new value
        final var accelerometerGenerator = new AccelerometerMeasurementsGenerator();

        creator.setAccelerometerNoiseRootPsdSource(accelerometerGenerator);

        // check
        assertSame(accelerometerGenerator, creator.getAccelerometerNoiseRootPsdSource());
    }

    @Test
    void testGetSetGyroscopeNoiseRootPsdSource() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertNull(creator.getGyroscopeNoiseRootPsdSource());

        // set new value
        final var gyroscopeGenerator = new GyroscopeMeasurementsGenerator();

        creator.sstGyroscopeNoiseRootPsdSource(gyroscopeGenerator);

        // check
        assertSame(gyroscopeGenerator, creator.getGyroscopeNoiseRootPsdSource());
    }

    @Test
    void testGetSetAccelerometerQuantizationLevel() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertEquals(0.0, creator.getAccelerometerQuantizationLevel(), 0.0);

        // set new value
        final var random = new Random();
        final var accelerometerQuantizationLevel = random.nextDouble();

        creator.setAccelerometerQuantizationLevel(accelerometerQuantizationLevel);

        // check
        assertEquals(accelerometerQuantizationLevel, creator.getAccelerometerQuantizationLevel(), 0.0);
    }

    @Test
    void testGetSetGyroQuantizationLevel() {
        final var creator = new IMUErrorsCreator();

        // check default value
        assertEquals(0.0, creator.getGyroQuantizationLevel(), 0.0);

        // set new value
        final var random = new Random();
        final var gyroscopeQuantizationLevel = random.nextDouble();

        creator.setGyroQuantizationLevel(gyroscopeQuantizationLevel);

        // check
        assertEquals(gyroscopeQuantizationLevel, creator.getGyroQuantizationLevel(), 0.0);
    }

    @Test
    void testCreate() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException, NotReadyException, RotationException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var random = new Random();
            final var randomizer = new UniformRandomizer(random);
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            final var gyroscopeMeasurements =
                    new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var accelerometerMeasurements = new ArrayList<StandardDeviationBodyKinematics>();

            final var generator = new AccelerometerAndGyroscopeMeasurementsGenerator(
                    new AccelerometerAndGyroscopeMeasurementsGeneratorListener() {
                        @Override
                        public void onInitializationStarted(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
                        }

                        @Override
                        public void onInitializationCompleted(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                final double accelerometerBaseNoiseLevel) {
                            // no action needed
                        }

                        @Override
                        public void onError(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                                final TriadStaticIntervalDetector.ErrorReason reason) {
                            // no action needed
                        }

                        @Override
                        public void onStaticIntervalDetected(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
                        }

                        @Override
                        public void onDynamicIntervalDetected(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
                        }

                        @Override
                        public void onStaticIntervalSkipped(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
                        }

                        @Override
                        public void onDynamicIntervalSkipped(
                                final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
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
                            gyroscopeMeasurements.add(measurement);
                        }

                        @Override
                        public void onReset(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                            // no action needed
                        }
                    });

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, random, 0);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, random, start);
                start += staticPeriodLength;

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, random, start, false);
                start += dynamicPeriodLength;
            }

            // as an initial value for gyroscope bias, we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var gyroCalibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                gyroCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBg = gyroCalibrator.getEstimatedBiases();
            final var estimatedMg = gyroCalibrator.getEstimatedMg();
            final var estimatedGg = gyroCalibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), 
                    accelerometerMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiases();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var estimatedAccelerometerNoiseRootPsd = generator.getAccelerometerBaseNoiseLevelRootPsd();
            final var estimatedGyroNoiseRootPsd = generator.getGyroscopeBaseNoiseLevelRootPsd();

            assertNotNull(estimatedBa);
            assertNotNull(estimatedMa);
            assertNotNull(estimatedBg);
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            final var accelerometerQuantizationLevel = random.nextDouble();
            final var gyroscopeQuantizationLevel = random.nextDouble();

            final var creator = new IMUErrorsCreator(accelerometerCalibrator, gyroCalibrator, generator, generator, 
                    accelerometerQuantizationLevel, gyroscopeQuantizationLevel);

            final var estimatedErrors = creator.create();

            assertNotNull(estimatedErrors);
            assertArrayEquals(estimatedBa, estimatedErrors.getAccelerometerBiases(), 0.0);
            assertArrayEquals(estimatedBg, estimatedErrors.getGyroBiases(), 0.0);
            assertEquals(estimatedMa, estimatedErrors.getAccelerometerScaleFactorAndCrossCouplingErrors());
            assertEquals(estimatedMg, estimatedErrors.getGyroScaleFactorAndCrossCouplingErrors());
            assertEquals(estimatedGg, estimatedErrors.getGyroGDependentBiases());
            assertEquals(estimatedAccelerometerNoiseRootPsd, estimatedErrors.getAccelerometerNoiseRootPSD(), 0.0);
            assertEquals(estimatedGyroNoiseRootPsd, estimatedErrors.getGyroNoiseRootPSD(), 0.0);
            assertEquals(accelerometerQuantizationLevel, estimatedErrors.getAccelerometerQuantizationLevel(), 0.0);
            assertEquals(gyroscopeQuantizationLevel, estimatedErrors.getGyroQuantizationLevel(), 0.0);

            if (!bg.equals(gyroCalibrator.getEstimatedBiasesAsMatrix(), ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(gyroCalibrator.getEstimatedBiasesAsMatrix(), ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(accelerometerCalibrator.getEstimatedBiasesAsMatrix(), SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(accelerometerCalibrator.getEstimatedBiasesAsMatrix(), SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR));

            assertTrue(estimatedAccelerometerNoiseRootPsd > 0.0);
            assertTrue(estimatedGyroNoiseRootPsd > 0.0);

            final var accelerometerNoiseLevel = generator.getAccelerometerBaseNoiseLevel();
            final var gyroNoiseLevel = generator.getGyroscopeBaseNoiseLevel();
            final var sqrtTimeInterval = Math.sqrt(generator.getTimeInterval());
            assertEquals(accelerometerNoiseLevel * sqrtTimeInterval, estimatedAccelerometerNoiseRootPsd,
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(gyroNoiseLevel * sqrtTimeInterval, estimatedGyroNoiseRootPsd, SMALL_ABSOLUTE_ERROR);

            assertEquals(SMALL_ROOT_PSD, estimatedAccelerometerNoiseRootPsd, SMALL_ABSOLUTE_ERROR);
            assertEquals(0.0, estimatedGyroNoiseRootPsd, SMALL_ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force NotReadyException
        final var creator = new IMUErrorsCreator();
        assertThrows(NotReadyException.class, creator::create);
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMaCommonAxis() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }

    private static void generateStaticSamples(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final IMUErrors errors, final Random random, final int startSample)
            throws LockedException {

        final var timedMeasuredKinematics = new TimedBodyKinematics();
        final var measuredKinematics = new BodyKinematics();
        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            assertTrue(generator.process(timedMeasuredKinematics));
        }
    }

    @SuppressWarnings("SameParameterValue")
    private static void generateDynamicSamples(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final Random random, final int startSample,
            final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException, RotationException {

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var nedC = nedFrame.getCoordinateTransformation();

        final var roll = nedC.getRollEulerAngle();
        final var pitch = nedC.getPitchEulerAngle();
        final var yaw = nedC.getYawEulerAngle();

        final var beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        var oldNedFrame = new NEDFrame(nedFrame);
        var newNedFrame = new NEDFrame();
        var oldEcefFrame = new ECEFFrame(ecefFrame);
        var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();

        final var timedMeasuredKinematics = new TimedBodyKinematics();
        final var measuredKinematics = new BodyKinematics();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final var progress = (double) i / (double) numSamples;

            final var newRoll = oldRoll + interpolate(deltaRoll, progress);
            final var newPitch = oldPitch + interpolate(deltaPitch, progress);
            final var newYaw = oldYaw + interpolate(deltaYaw, progress);
            final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final var newEcefX = oldEcefX + interpolate(deltaX, progress);
            final var newEcefY = oldEcefY + interpolate(deltaY, progress);
            final var newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame, 
                    trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            assertTrue(generator.process(timedMeasuredKinematics));

            final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(new BodyKinematics(trueKinematics),
                    timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final var afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ);

        final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);


        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after the dynamic sequence finishes, update true kinematics for a
        // static sequence at the current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // a dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private static double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }
}
