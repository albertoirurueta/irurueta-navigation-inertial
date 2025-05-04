/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.GNSSKalmanEpochEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class INSTightlyCoupledKalmanEpochEstimatorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -50.0;
    private static final double MAX_HEIGHT_METERS = 50.0;

    private static final double MIN_SPEED_VALUE = -2.0;
    private static final double MAX_SPEED_VALUE = 2.0;

    private static final int MIN_MEASUREMENTS = 4;
    private static final int MAX_MEASUREMENTS = 10;

    private static final double MIN_SAT_HEIGHT_METERS = 150000;
    private static final double MAX_SAT_HEIGHT_METERS = 500000;

    private static final double MIN_SAT_SPEED_VALUE = -20.0;
    private static final double MAX_SAT_SPEED_VALUE = 20.0;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    void testEstimate() throws AlgebraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

            final var userVn = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final var userVe = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);
            final var userVd = randomizer.nextDouble(MIN_SPEED_VALUE, MAX_SPEED_VALUE);

            final var userNedPosition = new NEDPosition(userLatitude, userLongitude, userHeight);
            final var userNedVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var userEcefPosition = new ECEFPosition();
            final var userEcefVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(userNedPosition, userNedVelocity, userEcefPosition,
                    userEcefVelocity);

            final var userNedFrame = new NEDFrame(userLatitude, userLongitude, userHeight, userVn, userVe, userVd);
            final var userEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(userNedFrame);

            final var userPosition = userEcefFrame.getPosition();

            final var numMeasurements = randomizer.nextInt(MIN_MEASUREMENTS, MAX_MEASUREMENTS);

            final var measurements = new ArrayList<GNSSMeasurement>();
            for (var i = 0; i < numMeasurements; i++) {
                final var satLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, 
                        MAX_LATITUDE_DEGREES));
                final var satLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, 
                        MAX_LONGITUDE_DEGREES));
                final var satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT_METERS, MAX_SAT_HEIGHT_METERS);

                final var satVn = randomizer.nextDouble(MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);
                final var satVe = randomizer.nextDouble(MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);
                final var satVd = randomizer.nextDouble(MIN_SAT_SPEED_VALUE, MAX_SAT_SPEED_VALUE);

                final var satNedFrame = new NEDFrame(satLatitude, satLongitude, satHeight, satVn, satVe, satVd);
                final var satEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(satNedFrame);

                final var satPosition = satEcefFrame.getPosition();

                final var pseudoRange = userPosition.distanceTo(satPosition);

                final var posDiff = new double[]{
                        satEcefFrame.getX() - userEcefFrame.getX(),
                        satEcefFrame.getY() - userEcefFrame.getY(),
                        satEcefFrame.getZ() - userEcefFrame.getZ()};
                final var posNorm = Utils.normF(posDiff);

                final var velDiff = new double[]{
                        satEcefFrame.getVx() - userEcefFrame.getVx(),
                        satEcefFrame.getVy() - userEcefFrame.getVy(),
                        satEcefFrame.getVz() - userEcefFrame.getVz()};
                final var velNorm = Utils.normF(velDiff);

                final var dot = Utils.dotProduct(posDiff, velDiff);
                final var cosAngle = dot / (posNorm * velNorm);

                final var pseudoRate = velNorm * cosAngle;

                final var x = satEcefFrame.getX();
                final var y = satEcefFrame.getY();
                final var z = satEcefFrame.getZ();

                final var vx = satEcefFrame.getVx();
                final var vy = satEcefFrame.getVy();
                final var vz = satEcefFrame.getVz();

                measurements.add(new GNSSMeasurement(pseudoRange, pseudoRate, x, y, z, vx, vy, vz));
            }

            final var accelerationBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerationBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerationBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var receiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var receiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

            final var covariance = Matrix.identity(INSTightlyCoupledKalmanState.NUM_PARAMS, 
                    INSTightlyCoupledKalmanState.NUM_PARAMS);

            final var bodyKinematics = new BodyKinematics();
            final var fx = bodyKinematics.getFx();
            final var fy = bodyKinematics.getFy();
            final var fz = bodyKinematics.getFz();

            final var previousPosition = new ECEFPosition(
                    userEcefPosition.getX() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVx(),
                    userEcefPosition.getY() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVy(),
                    userEcefPosition.getZ() + TIME_INTERVAL_SECONDS * userEcefVelocity.getVz());

            final var previousNedPosition = new NEDPosition();
            final var previousNedVelocity = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(previousPosition, userEcefVelocity, previousNedPosition,
                    previousNedVelocity);
            final var previousLatitude = previousNedPosition.getLatitude();

            final var previousState = new INSTightlyCoupledKalmanState(
                    userEcefFrame.getCoordinateTransformation(), userEcefVelocity, previousPosition,
                    accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                    receiverClockOffset, receiverClockDrift, covariance);

            final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
            final var config = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, 
                    accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

            final var result1 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, previousLatitude, config, result1);

            final var result2 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitude, config);

            final var propagationInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);

            final var result3 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState, fx, fy, fz,
                    previousLatitude, config, result3);

            final var result4 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, 
                    previousState, fx, fy, fz, previousLatitude, config);

            final var result5 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, config, result5);

            final var result6 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, config);

            final var result7 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState, fx, fy, fz,
                    config, result7);

            final var result8 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, 
                    previousState, fx, fy, fz, config);

            final var result9 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitude, config, result9);

            final var result10 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitude, config);

            final var result11 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState, 
                    bodyKinematics, previousLatitude, config, result11);

            final var result12 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, previousLatitude, config);

            final var result13 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, config, result13);

            final var result14 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, config);

            final var result15 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState,
                    bodyKinematics, config, result15);

            final var result16 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, config);

            final var previousLatitudeAngle = new Angle(previousLatitude, AngleUnit.RADIANS);
            final var result17 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    fx, fy, fz, previousLatitudeAngle, config, result17);

            final var result18 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var result19 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState, fx, fy, fz,
                    previousLatitudeAngle, config, result19);

            final var result20 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, fx, fy, fz, previousLatitudeAngle, config);

            final var result21 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS, previousState,
                    bodyKinematics, previousLatitudeAngle, config, result21);

            final var result22 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, TIME_INTERVAL_SECONDS,
                    previousState, bodyKinematics, previousLatitudeAngle, config);

            final var result23 = new INSTightlyCoupledKalmanState();
            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, previousState,
                    bodyKinematics, previousLatitudeAngle, config, result23);

            final var result24 = INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval,
                    previousState, bodyKinematics, previousLatitudeAngle, config);

            assertEquals(result1, result2);
            assertEquals(result1, result3);
            assertEquals(result1, result4);
            assertEquals(result1, result5);
            assertEquals(result1, result6);
            assertEquals(result1, result7);
            assertEquals(result1, result8);
            assertEquals(result1, result9);
            assertEquals(result1, result10);
            assertEquals(result1, result11);
            assertEquals(result1, result12);
            assertEquals(result1, result13);
            assertEquals(result1, result14);
            assertEquals(result1, result15);
            assertEquals(result1, result16);
            assertEquals(result1, result17);
            assertEquals(result1, result18);
            assertEquals(result1, result19);
            assertEquals(result1, result20);
            assertEquals(result1, result21);
            assertEquals(result1, result22);
            assertEquals(result1, result23);
            assertEquals(result1, result24);

            final var expected = estimate(measurements, previousState, fx, fy, fz, previousLatitude, config);

            if (!expected.equals(result1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(expected.equals(result1, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static INSTightlyCoupledKalmanState estimate(
            final List<GNSSMeasurement> measurements, final INSTightlyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final double previousLatitude,
            final INSTightlyCoupledKalmanConfig config) throws AlgebraException {

        // Skew symmetric matrix of Earth rate
        final var omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, Constants.EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final var phiMatrix = Matrix.identity(17, 17);
        phiMatrix.setSubmatrix(0, 0, 2, 2,
                phiMatrix.getSubmatrix(0, 0, 2, 2)
                        .subtractAndReturnNew(omegaIe.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS)));

        final var estCbeOld = previousState.getBodyToEcefCoordinateTransformationMatrix();
        phiMatrix.setSubmatrix(0, 12, 2, 14,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        final var measFibb = Matrix.newFromArray(new double[]{fx, fy, fz});
        phiMatrix.setSubmatrix(3, 0, 5, 2,
                Utils.skewMatrix(estCbeOld.multiplyAndReturnNew(measFibb))
                        .multiplyByScalarAndReturnNew(-TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(3, 3, 5, 5,
                phiMatrix.getSubmatrix(3, 3, 5, 5)
                        .subtractAndReturnNew(omegaIe.multiplyByScalarAndReturnNew(2.0 * TIME_INTERVAL_SECONDS)));

        final var sinPrevLat = Math.sin(previousLatitude);
        final var cosPrevLat = Math.cos(previousLatitude);
        final var sinPrevLat2 = sinPrevLat * sinPrevLat;
        final var cosPrevLat2 = cosPrevLat * cosPrevLat;

        final var geocentricRadius = Constants.EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(Constants.EARTH_ECCENTRICITY * sinPrevLat, 2.0))
                * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - Constants.EARTH_ECCENTRICITY * Constants.EARTH_ECCENTRICITY, 2.0)
                * sinPrevLat2);

        final var prevX = previousState.getX();
        final var prevY = previousState.getY();
        final var prevZ = previousState.getZ();
        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(prevX, prevY, prevZ);
        final var g = gravity.asMatrix();

        final var estRebeOld = Matrix.newFromArray(new double[]{prevX, prevY, prevZ});

        final var gScaled = g.multiplyByScalarAndReturnNew(-2.0 * TIME_INTERVAL_SECONDS / geocentricRadius);
        final var estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        final var previousPositionNorm = Math.sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);
        final var estRebeOldTransScaled = estRebeOldTrans.multiplyByScalarAndReturnNew(1.0 / previousPositionNorm);
        phiMatrix.setSubmatrix(3, 6, 5, 8,
                gScaled.multiplyAndReturnNew(estRebeOldTransScaled));

        phiMatrix.setSubmatrix(3, 9, 5, 11,
                estCbeOld.multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        phiMatrix.setSubmatrix(6, 3, 8, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(TIME_INTERVAL_SECONDS));

        phiMatrix.setElementAt(15, 16, TIME_INTERVAL_SECONDS);

        // 2. Determine approximate system noise covariance matrix using (14.82)
        final var qPrimeMatrix = new Matrix(17, 17);
        qPrimeMatrix.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(3, 3, 5, 5,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerNoisePSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(9, 9, 11, 11,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getAccelerometerBiasPSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setSubmatrix(12, 12, 14, 14,
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        config.getGyroBiasPSD() * TIME_INTERVAL_SECONDS));
        qPrimeMatrix.setElementAt(15, 15, config.getClockPhasePSD() * TIME_INTERVAL_SECONDS);
        qPrimeMatrix.setElementAt(16, 16, config.getClockFrequencyPSD() * TIME_INTERVAL_SECONDS);

        // 3. Propagate state estimates using (3.14) noting that only the clock
        // states are non-zero due to closed-loop correction.
        final var xEstPropagated = new Matrix(17, 1);
        xEstPropagated.setElementAtIndex(15, previousState.getReceiverClockOffset()
                + previousState.getReceiverClockDrift() * TIME_INTERVAL_SECONDS);
        xEstPropagated.setElementAtIndex(16, previousState.getReceiverClockDrift());

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final var pMatrixOld = previousState.getCovariance();
        final var phiTrans = phiMatrix.transposeAndReturnNew();
        final var halfQ = qPrimeMatrix.multiplyByScalarAndReturnNew(0.5);
        final var tmp1 = pMatrixOld.addAndReturnNew(halfQ);
        final var pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp1);
        pMatrixPropagated.multiply(phiTrans);
        pMatrixPropagated.add(halfQ);

        // MEASUREMENT UPDATE PHASE

        final var noMeas = measurements.size();
        final var uAseT = new Matrix(noMeas, 3);
        final var predMeas = new Matrix(noMeas, 2);

        final var prevVx = previousState.getVx();
        final var prevVy = previousState.getVy();
        final var prevVz = previousState.getVz();
        final var estVebeOld = Matrix.newFromArray(new double[]{prevVx, prevVy, prevVz});

        // Loop measurements
        for (var j = 0; j < noMeas; j++) {
            // Predict approx range
            final var measurement = measurements.get(j);
            var deltaR = Matrix.newFromArray(new double[]{
                    measurement.getX() - prevX,
                    measurement.getY() - prevY,
                    measurement.getZ() - prevZ
            });

            final var approxRange = Math.sqrt(Utils.dotProduct(deltaR.transposeAndReturnNew(), deltaR));

            // Calculate frame rotation during signal transit time using (8.36)
            final var cei = new Matrix(3, 3);
            cei.fromArray(new double[]{
                    1.0, Constants.EARTH_ROTATION_RATE * approxRange / GNSSKalmanEpochEstimator.SPEED_OF_LIGHT, 0.0,
                    -Constants.EARTH_ROTATION_RATE * approxRange / GNSSKalmanEpochEstimator.SPEED_OF_LIGHT, 1.0, 0.0,
                    0.0, 0.0, 1.0}, false);

            // Predict pseudo-range using (9.165)
            final var measurementPosition = Matrix.newFromArray(new double[]{
                    measurement.getX(), measurement.getY(), measurement.getZ()});
            final var measurementVelocity = Matrix.newFromArray(new double[]{
                    measurement.getVx(), measurement.getVy(), measurement.getVz()});
            deltaR = cei.multiplyAndReturnNew(measurementPosition);
            deltaR.subtract(estRebeOld);
            final var range = Math.sqrt(Utils.dotProduct(deltaR.transposeAndReturnNew(), deltaR));
            predMeas.setElementAt(j, 0, range + xEstPropagated.getElementAtIndex(15));

            // Predict line of sight
            uAseT.setSubmatrix(j, 0, j, 2,
                    deltaR.transposeAndReturnNew().multiplyByScalarAndReturnNew(1.0 / range));

            // Predict pseudo-range rate using (9.165)
            final var rangeRate = uAseT.getSubmatrix(j, 0, j, 2).multiplyAndReturnNew(
                    cei.multiplyAndReturnNew(measurementVelocity.addAndReturnNew(
                            omegaIe.multiplyAndReturnNew(measurementPosition)))
                            .subtractAndReturnNew(estVebeOld.addAndReturnNew(
                                    omegaIe.multiplyAndReturnNew(estRebeOld))));

            predMeas.setElementAt(j, 1, rangeRate.getElementAtIndex(0)
                    + xEstPropagated.getElementAtIndex(16));
        }

        // 5. Set up a measurement matrix using (14.126)
        final var hMatrix = new Matrix(2 * noMeas, 17);
        hMatrix.setSubmatrix(0, 6, noMeas - 1, 8,
                uAseT.getSubmatrix(0, 0, noMeas - 1, 2));
        final var ones = new double[noMeas];
        Arrays.fill(ones, 1.0);
        hMatrix.setSubmatrix(0, 15, noMeas - 1, 15, ones);
        hMatrix.setSubmatrix(noMeas, 3, 2 * noMeas - 1, 5,
                uAseT.getSubmatrix(0, 0, noMeas - 1, 2));
        hMatrix.setSubmatrix(noMeas, 16, 2 * noMeas - 1, 16, ones);

        // 6. Set up a measurement noise covariance matrix assuming all measurements
        // are independent and have equal variance for a given measurement type.
        final var rMatrix = new Matrix(2 * noMeas, 2 * noMeas);
        rMatrix.setSubmatrix(0, 0, noMeas - 1, noMeas - 1,
                Matrix.identity(noMeas, noMeas).multiplyByScalarAndReturnNew(Math.pow(config.getPseudoRangeSD(), 2.0)));
        rMatrix.setSubmatrix(0, noMeas, noMeas - 1, 2 * noMeas - 1,
                new Matrix(noMeas, noMeas));
        rMatrix.setSubmatrix(noMeas, 0, 2 * noMeas - 1, noMeas - 1,
                new Matrix(noMeas, noMeas));
        rMatrix.setSubmatrix(noMeas, noMeas, 2 * noMeas - 1, 2 * noMeas - 1,
                Matrix.identity(noMeas, noMeas).multiplyByScalarAndReturnNew(Math.pow(config.getRangeRateSD(), 2.0)));

        // 7. Calculate Kalman gain using (3.21)
        final var kMatrix = pMatrixPropagated.multiplyAndReturnNew(
                hMatrix.transposeAndReturnNew().multiplyAndReturnNew(
                        Utils.inverse(hMatrix.multiplyAndReturnNew(pMatrixPropagated)
                                .multiplyAndReturnNew(hMatrix.transposeAndReturnNew())
                                .addAndReturnNew(rMatrix))));

        // 8. Formulate measurement innovations using (14.119)
        final var deltaZ = new Matrix(2 * noMeas, 1);
        for (var j = 0; j < noMeas; j++) {
            final var measurement = measurements.get(j);
            deltaZ.setElementAtIndex(j, measurement.getPseudoRange() - predMeas.getElementAtIndex(j));
            deltaZ.setElementAtIndex(noMeas + j,
                    measurement.getPseudoRate() - predMeas.getElementAtIndex(noMeas + j));
        }

        // 9. Update state estimates using (3.24)
        final var xEstNew = xEstPropagated.addAndReturnNew(kMatrix.multiplyAndReturnNew(deltaZ));

        // 10. Update state estimation error covariance matrix using (3.25)
        final var pMatrixNew = Matrix.identity(17, 17)
                .subtractAndReturnNew(kMatrix.multiplyAndReturnNew(hMatrix))
                .multiplyAndReturnNew(pMatrixPropagated);

        // CLOSED-LOOP CORRECTION

        // Correct attitude, velocity, and position using (14.7-9)
        final var estCbeNew = Matrix.identity(3, 3).subtractAndReturnNew(Utils.skewMatrix(
                xEstNew.getSubmatrix(0, 0, 2, 0)));
        estCbeNew.multiply(estCbeOld);
        final var estVebeNew = estVebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(3, 0, 5, 0));
        final var estRebeNew = estRebeOld.subtractAndReturnNew(
                xEstNew.getSubmatrix(6, 0, 8, 0));

        // Update IMU bias and GNSS receiver clock estimates
        final var estIMUbiasOld = Matrix.newFromArray(new double[]{
                previousState.getAccelerationBiasX(),
                previousState.getAccelerationBiasY(),
                previousState.getAccelerationBiasZ(),
                previousState.getGyroBiasX(),
                previousState.getGyroBiasY(),
                previousState.getGyroBiasZ()
        });
        final var estIMUbiasNew = estIMUbiasOld.addAndReturnNew(
                xEstNew.getSubmatrix(9, 0, 14, 0));

        final var estClockNew = xEstNew.getSubmatrix(
                15, 0, 16, 0);

        return new INSTightlyCoupledKalmanState(estCbeNew,
                estVebeNew.getElementAtIndex(0),
                estVebeNew.getElementAtIndex(1),
                estVebeNew.getElementAtIndex(2),
                estRebeNew.getElementAtIndex(0),
                estRebeNew.getElementAtIndex(1),
                estRebeNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(0),
                estIMUbiasNew.getElementAtIndex(1),
                estIMUbiasNew.getElementAtIndex(2),
                estIMUbiasNew.getElementAtIndex(3),
                estIMUbiasNew.getElementAtIndex(4),
                estIMUbiasNew.getElementAtIndex(5),
                estClockNew.getElementAtIndex(0),
                estClockNew.getElementAtIndex(1),
                pMatrixNew);
    }
}
