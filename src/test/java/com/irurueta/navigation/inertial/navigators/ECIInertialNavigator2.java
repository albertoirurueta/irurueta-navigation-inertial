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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.navigation.inertial.estimators.ECIGravitationEstimator;

/**
 * This is an implementation of {@link ECIInertialNavigator} for testing purposes only.
 */
class ECIInertialNavigator2 {

    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final int ROWS = 3;

    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), kinematics,
                    result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToEciCoordinateTransformationMatrix(oldC)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        try {
            // Attitude update
            // Calculate attitude increment, magnitude, and skew-symmetric matrix
            final Matrix alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0, angularRateX * timeInterval);
            alphaIbb.setElementAtIndex(1, angularRateY * timeInterval);
            alphaIbb.setElementAtIndex(2, angularRateZ * timeInterval);

            final double magAlpha = Utils.normF(alphaIbb);

            final Matrix skewAlphaIbb = Utils.skewMatrix(alphaIbb);

            // Obtain coordinate transformation matrix from the new attitude to the old
            // using Rodrigues' formula, (5.73)
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

            // Update attitude
            final Matrix oldCbi = oldC.getMatrix();
            final Matrix cbi = oldCbi.multiplyAndReturnNew(cNewOld);

            // Specific force frame transformation
            // Calculate the average body-to-ECI-frame coordinate transformation
            // matrix over the update interval using (5.84)
            final Matrix aveCbi;
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                final double value2 = (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2;

                aveCbi = oldCbi.multiplyAndReturnNew(
                        Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                                .addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(value1))
                                .addAndReturnNew(skewAlphaIbb2.multiplyByScalarAndReturnNew(value2)));
            } else {
                aveCbi = oldCbi;
            }

            // Transform specific force to ECI-frame resolving axes using (5.81)
            final Matrix fIbb = new Matrix(ROWS, 1);
            fIbb.setElementAtIndex(0, fx);
            fIbb.setElementAtIndex(1, fy);
            fIbb.setElementAtIndex(2, fz);

            final Matrix fibi = aveCbi.multiplyAndReturnNew(fIbb);

            // Update velocity
            // From (5.18) and (5.20),
            final Matrix oldVibi = new Matrix(ROWS, 1);
            oldVibi.setElementAtIndex(0, oldVx);
            oldVibi.setElementAtIndex(1, oldVy);
            oldVibi.setElementAtIndex(2, oldVz);

            final ECIGravitation gravitation = ECIGravitationEstimator
                    .estimateGravitationAndReturnNew(oldX, oldY, oldZ);
            final Matrix g = gravitation.asMatrix();

            final Matrix vIbi = oldVibi.addAndReturnNew(
                    fibi.addAndReturnNew(g).multiplyByScalarAndReturnNew(timeInterval));

            final double vx = vIbi.getElementAtIndex(0);
            final double vy = vIbi.getElementAtIndex(1);
            final double vz = vIbi.getElementAtIndex(2);

            // Update cartesian position
            // From (5.23),
            final double x = oldX + (vx + oldVx) * 0.5 * timeInterval;
            final double y = oldY + (vy + oldVy) * 0.5 * timeInterval;
            final double z = oldZ + (vz + oldVz) * 0.5 * timeInterval;

            final CoordinateTransformation newC = new CoordinateTransformation(cbi,
                    FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            result.setCoordinates(x, y, z);
            result.setVelocityCoordinates(vx, vy, vz);
            result.setCoordinateTransformation(newC);

        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new InertialNavigatorException(e);
        }
    }

    public static boolean isValidBodyToEciCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return ECIFrame.isValidCoordinateTransformation(c);
    }
}
