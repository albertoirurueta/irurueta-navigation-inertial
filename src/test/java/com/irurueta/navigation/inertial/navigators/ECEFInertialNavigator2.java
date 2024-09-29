/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.navigators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameException;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;

/**
 * This is an implementation of {@link ECEFInertialNavigator} for testing purposes only.
 */
class ECEFInertialNavigator2 {

    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final int ROWS = 3;

    public static void navigateECEF(
            final double timeInterval, final ECEFFrame oldFrame, final BodyKinematics kinematics,
            final ECEFFrame result) throws InertialNavigatorException {
        try {
            navigateECEF(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(), oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(),
                    kinematics, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    public static void navigateECEF(
            final double timeInterval, final double oldX, final double oldY, final double oldZ,
            final CoordinateTransformation oldC, final double oldVx, final double oldVy, final double oldVz,
            final BodyKinematics kinematics, final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {
        navigateECEF(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), result);
    }

    public static void navigateECEF(
            final double timeInterval, final double oldX, final double oldY, final double oldZ,
            final CoordinateTransformation oldC, final double oldVx, final double oldVy, final double oldVz,
            final double fx, final double fy, final double fz, final double angularRateX, final double angularRateY,
            final double angularRateZ, final ECEFFrame result) throws InertialNavigatorException,
            InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToEcefCoordinateTransformationMatrix(oldC)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        try {
            // Attitude update
            // From (2.145) determine the Earth rotation over the update interval
            final double alphaIe = EARTH_ROTATION_RATE * timeInterval;
            final double sinAlpha = Math.sin(alphaIe);
            final double cosAlpha = Math.cos(alphaIe);
            final Matrix cEarth = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS);
            cEarth.setElementAt(0, 0, cosAlpha);
            cEarth.setElementAt(0, 1, sinAlpha);
            cEarth.setElementAt(0, 2, 0.0);
            cEarth.setElementAt(1, 0, -sinAlpha);
            cEarth.setElementAt(1, 1, cosAlpha);
            cEarth.setElementAt(1, 2, 0.0);
            cEarth.setElementAt(2, 0, 0.0);
            cEarth.setElementAt(2, 1, 0.0);
            cEarth.setElementAt(2, 2, 1.0);

            // Calculate attitude increment, magnitude, and skew-symmetric matrix
            final Matrix alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0, angularRateX * timeInterval);
            alphaIbb.setElementAtIndex(1, angularRateY * timeInterval);
            alphaIbb.setElementAtIndex(2, angularRateZ * timeInterval);

            final double magAlpha = Utils.normF(alphaIbb);

            final Matrix skewAlphaIbb = Utils.skewMatrix(alphaIbb);

            // Obtain coordinate transformation matrix from the new attitude with
            // respect an inertial frame to the old using Rodrigues' formula, (5.73)
            final Matrix cNewOld;
            final Matrix skewAlphaIbb2;
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = Math.sin(magAlpha) / magAlpha;
                final double value2 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                skewAlphaIbb2 = skewAlphaIbb.multiplyAndReturnNew(skewAlphaIbb);
                cNewOld = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                        .addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(value1))
                        .addAndReturnNew(skewAlphaIbb2.multiplyByScalarAndReturnNew(value2));
            } else {
                skewAlphaIbb2 = null;
                cNewOld = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                        .addAndReturnNew(skewAlphaIbb);
            }

            // Update attitude using (5.75)
            final Matrix oldCbe = oldC.getMatrix();
            final Matrix cbe = cEarth.multiplyAndReturnNew(oldCbe).multiplyAndReturnNew(cNewOld);

            // Specific force frame transformation
            // Calculate the average body-to-ECEF-frame coordinate transformation
            // matrix over the update interval using (5.84) and (5.85).
            final Matrix aveCbe;
            final Matrix skewAlphaIe = Utils.skewMatrix(new double[]{0.0, 0.0, alphaIe});
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                final double value2 = (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2;

                aveCbe = oldCbe.multiplyAndReturnNew(
                                Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                                        .addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(value1))
                                        .addAndReturnNew(skewAlphaIbb2
                                                .multiplyByScalarAndReturnNew(value2)))
                        .subtractAndReturnNew(skewAlphaIe.multiplyAndReturnNew(oldCbe)
                                .multiplyByScalarAndReturnNew(0.5));
            } else {
                aveCbe = oldCbe.subtractAndReturnNew(skewAlphaIe.multiplyAndReturnNew(oldCbe)
                        .multiplyByScalarAndReturnNew(0.5));
            }

            // Transform specific force to ECEF-frame resolving axes using (5.85)
            final Matrix fibb = new Matrix(ROWS, 1);
            fibb.setElementAtIndex(0, fx);
            fibb.setElementAtIndex(1, fy);
            fibb.setElementAtIndex(2, fz);

            final Matrix fibe = aveCbe.multiplyAndReturnNew(fibb);

            // Update velocity
            // From (5.36)
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldX, oldY, oldZ);
            final Matrix g = gravity.asMatrix();

            final Matrix oldVebe = new Matrix(ROWS, 1);
            oldVebe.setElementAtIndex(0, oldVx);
            oldVebe.setElementAtIndex(1, oldVy);
            oldVebe.setElementAtIndex(2, oldVz);

            final Matrix skewOmegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});
            final Matrix vebe = oldVebe.addAndReturnNew(
                    fibe.addAndReturnNew(g).subtractAndReturnNew(
                                    skewOmegaIe.multiplyAndReturnNew(oldVebe)
                                            .multiplyByScalarAndReturnNew(2.0))
                            .multiplyByScalarAndReturnNew(timeInterval));

            final double newVx = vebe.getElementAtIndex(0);
            final double newVy = vebe.getElementAtIndex(1);
            final double newVz = vebe.getElementAtIndex(2);

            // Update cartesian position
            // From (5.38)
            final double newX = oldX + (newVx + oldVx) * 0.5 * timeInterval;
            final double newY = oldY + (newVy + oldVy) * 0.5 * timeInterval;
            final double newZ = oldZ + (newVz + oldVz) * 0.5 * timeInterval;

            final CoordinateTransformation newC = new CoordinateTransformation(FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            newC.setMatrix(cbe);
            result.setCoordinateTransformation(newC);
            result.setVelocityCoordinates(newVx, newVy, newVz);
            result.setCoordinates(newX, newY, newZ);
        } catch (final AlgebraException | FrameException | InvalidRotationMatrixException e) {
            throw new InertialNavigatorException(e);
        }
    }

    public static boolean isValidBodyToEcefCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return ECEFFrame.isValidCoordinateTransformation(c);
    }
}
