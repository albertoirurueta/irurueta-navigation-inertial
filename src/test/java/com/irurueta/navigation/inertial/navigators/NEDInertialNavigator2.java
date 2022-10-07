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
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator;
import com.irurueta.navigation.inertial.estimators.RadiiOfCurvatureEstimator;

/**
 * This is an implementation of {@link NEDInertialNavigator} for testing purposes only.
 */
public class NEDInertialNavigator2 {
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    public static final double DEFAULT_ACCURACY_THRESHOLD = CoordinateTransformation.DEFAULT_THRESHOLD;

    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final int ROWS = 3;

    public static void navigateNED(final double timeInterval,
                                   final NEDFrame oldFrame,
                                   final BodyKinematics kinematics,
                                   final double accuracyThreshold,
                                   final NEDFrame result)
            throws InertialNavigatorException {
        try {
            navigateNED(timeInterval, oldFrame.getLatitude(), oldFrame.getLongitude(),
                    oldFrame.getHeight(), oldFrame.getCoordinateTransformation(),
                    oldFrame.getVn(), oldFrame.getVe(), oldFrame.getVd(), kinematics,
                    accuracyThreshold, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    public static void navigateNED(final double timeInterval,
                                   final double oldLatitude,
                                   final double oldLongitude,
                                   final double oldHeight,
                                   final CoordinateTransformation oldC,
                                   final double oldVn,
                                   final double oldVe,
                                   final double oldVd,
                                   final BodyKinematics kinematics,
                                   final double accuracyThreshold,
                                   final NEDFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        navigateNED(timeInterval, oldLatitude, oldLongitude, oldHeight, oldC,
                oldVn, oldVe, oldVd, kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(), kinematics.getAngularRateZ(), accuracyThreshold, result);
    }

    public static void navigateNED(final double timeInterval,
                                   final double oldLatitude,
                                   final double oldLongitude,
                                   final double oldHeight,
                                   final CoordinateTransformation oldC,
                                   final double oldVn,
                                   final double oldVe,
                                   final double oldVd,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final double accuracyThreshold,
                                   final NEDFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToNEDCoordinateTransformationMatrix(oldC)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        try {
            // Calculate attitude increment, magnitude, and skew-symmetric matrix
            final Matrix alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0, angularRateX * timeInterval);
            alphaIbb.setElementAtIndex(1, angularRateY * timeInterval);
            alphaIbb.setElementAtIndex(2, angularRateZ * timeInterval);

            final double magAlpha = Utils.normF(alphaIbb);

            final Matrix skewAlphaIbb = Utils.skewMatrix(alphaIbb);

            // From (2.123), determine the angular rate of the ECEF frame with respect
            // the ECI frame, resolved about NED
            // From (2.123), determine the angular rate of the ECEF frame with respect
            // the ECI frame, resolved about NED
            final Matrix omegaIen = new Matrix(ROWS, 1);
            omegaIen.setElementAtIndex(0,
                    Math.cos(oldLatitude) * EARTH_ROTATION_RATE);
            omegaIen.setElementAtIndex(2,
                    -Math.sin(oldLatitude) * EARTH_ROTATION_RATE);

            // From (5.44), determine the angular rate of the NED frame with respect
            // the ECEF frame, resolved about NED
            final RadiiOfCurvature oldRadiiOfCurvature = RadiiOfCurvatureEstimator
                    .estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
            final double oldRe = oldRadiiOfCurvature.getRe();
            final double oldRn = oldRadiiOfCurvature.getRn();

            final Matrix oldOmegaEnN = new Matrix(ROWS, 1);
            oldOmegaEnN.setElementAtIndex(0, oldVe / (oldRe + oldHeight));
            oldOmegaEnN.setElementAtIndex(1, -oldVn / (oldRn + oldHeight));
            oldOmegaEnN.setElementAtIndex(2, -oldVe * Math.tan(oldLatitude) / (oldRe + oldHeight));

            final Matrix oldCbn = oldC.getMatrix();

            // Specific force frame transformation
            // Calculate the average body-to-ECEF-frame coordinate transformation
            // matrix over the update interval using (5.84) and (5.86)
            final Matrix aveCbn;
            final Matrix skewAlphaIbb2;
            final Matrix skewOmega = Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen));
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                final double value2 = (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2;
                skewAlphaIbb2 = skewAlphaIbb.multiplyAndReturnNew(skewAlphaIbb);
                aveCbn = oldCbn.multiplyAndReturnNew(
                        Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                                .addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(value1))
                                .addAndReturnNew(skewAlphaIbb2
                                        .multiplyByScalarAndReturnNew(value2)))
                        .subtractAndReturnNew(skewOmega.multiplyAndReturnNew(oldCbn)
                                .multiplyByScalarAndReturnNew(0.5));
            } else {
                skewAlphaIbb2 = null;
                aveCbn = oldCbn.subtractAndReturnNew(skewOmega.multiplyAndReturnNew(oldCbn)
                        .multiplyByScalarAndReturnNew(0.5));
            }

            // Transform specific force to ECEF-frame resolving axes using (5.86)
            final Matrix fIbb = new Matrix(ROWS, 1);
            fIbb.setElementAtIndex(0, fx);
            fIbb.setElementAtIndex(1, fy);
            fIbb.setElementAtIndex(2, fz);

            final Matrix fibn = aveCbn.multiplyAndReturnNew(fIbb);

            // Update velocity
            // From (5.54),
            final NEDGravity gravity = NEDGravityEstimator
                    .estimateGravityAndReturnNew(oldLatitude, oldHeight);
            final Matrix g = gravity.asMatrix();

            final Matrix oldVebn = new Matrix(ROWS, 1);
            oldVebn.setElementAtIndex(0, oldVn);
            oldVebn.setElementAtIndex(1, oldVe);
            oldVebn.setElementAtIndex(2, oldVd);

            final Matrix skewOmega2 = Utils.skewMatrix(
                    oldOmegaEnN.addAndReturnNew(
                            omegaIen.multiplyByScalarAndReturnNew(2.0)));

            final Matrix vEbn = oldVebn.addAndReturnNew(
                    fibn.addAndReturnNew(g).subtractAndReturnNew(
                            skewOmega2.multiplyAndReturnNew(oldVebn))
                            .multiplyByScalarAndReturnNew(timeInterval));

            final double vn = vEbn.getElementAtIndex(0);
            final double ve = vEbn.getElementAtIndex(1);
            final double vd = vEbn.getElementAtIndex(2);

            // Update curvilinear position
            // Update height using (5.56)
            final double height = oldHeight - 0.5 * timeInterval * (oldVd + vd);

            // Update latitude using (5.56)
            final double latitude = oldLatitude
                    + 0.5 * timeInterval * (oldVn / (oldRn + oldHeight) + vn / (oldRn + height));

            // Calculate meridian and transverse radii of curvature
            final RadiiOfCurvature radiiOfCurvature = RadiiOfCurvatureEstimator
                    .estimateRadiiOfCurvatureAndReturnNew(latitude);
            final double rn = radiiOfCurvature.getRn();
            final double re = radiiOfCurvature.getRe();

            // Update longitude using (5.56)
            final double longitude = oldLongitude
                    + 0.5 * timeInterval * (oldVe / ((oldRe + oldHeight) * Math.cos(oldLatitude))
                    + ve / ((re + height) * Math.cos(latitude)));

            // Attitude update
            // From (5.44), determine the angular rate of the NED frame with respect the
            // ECEF frame, resolved about NED
            final Matrix omegaEnN = new Matrix(ROWS, 1);
            omegaEnN.setElementAtIndex(0, ve / (re + height));
            omegaEnN.setElementAtIndex(1, -vn / (rn + height));
            omegaEnN.setElementAtIndex(2, -ve * Math.tan(latitude) / (re + height));

            // Obtain coordinate transformation matrix from the new attitude with respect
            // an inertial frame to the old using Rodrigues' formula, (5.73)
            final Matrix cNewOld;
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = Math.sin(magAlpha) / magAlpha;
                final double value2 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                cNewOld = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                        .addAndReturnNew(skewAlphaIbb.multiplyByScalarAndReturnNew(value1))
                        .addAndReturnNew(skewAlphaIbb2.multiplyByScalarAndReturnNew(value2));
            } else {
                cNewOld = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                        .addAndReturnNew(skewAlphaIbb);
            }

            // Update attitude using (5.77)
            final Matrix skewOmega3 = Utils.skewMatrix(
                    omegaIen.addAndReturnNew(omegaEnN.multiplyByScalarAndReturnNew(0.5))
                            .addAndReturnNew(oldOmegaEnN.multiplyByScalarAndReturnNew(0.5)));
            final Matrix cbn = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
                    .subtractAndReturnNew(skewOmega3.multiplyByScalarAndReturnNew(timeInterval))
                    .multiplyAndReturnNew(oldCbn)
                    .multiplyAndReturnNew(cNewOld);

            final CoordinateTransformation c = new CoordinateTransformation(cbn,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME, accuracyThreshold);

            result.setPosition(latitude, longitude, height);
            result.setVelocityCoordinates(vn, ve, vd);
            result.setCoordinateTransformation(c);

        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new InertialNavigatorException(e);
        }
    }

    public static boolean isValidBodyToNEDCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return NEDFrame.isValidCoordinateTransformation(c);
    }
}
