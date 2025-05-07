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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyKinematics;

/**
 * This implementation provides slightly more accurate
 * roll and pitch attitude angles than the ones obtained by
 * {@link LevelingEstimator}, since north component of gravity in a
 * local navigation frame is not neglected, because Earth is not
 * considered to be fully spherical.
 * <p>
 * To get this slight improvement of accuracy, this estimator requires
 * knowledge of device position (latitude and height) on Earth.
 *
 * @see LevelingEstimator
 */
public class LevelingEstimator2 {

    /**
     * Private constructor to prevent instantiation.
     */
    private LevelingEstimator2() {
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param latitude     device latitude expressed in radians (rad).
     * @param height       device height expressed in meters (m).
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param result       instance where attitude will be stored.
     */
    public static void getAttitude(
            final double latitude, final double height,
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY, final double angularRateZ,
            final CoordinateTransformation result) {

        getPartialAttitude(latitude, height, fx, fy, fz, result);

        // fix yaw angle
        final var roll = result.getRollEulerAngle();
        final var pitch = result.getPitchEulerAngle();
        final var yaw = LevelingEstimator.getYaw(roll, pitch, angularRateX, angularRateY, angularRateZ);

        result.setEulerAngles(roll, pitch, yaw);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param position     device position expressed in NED frame.
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param result       instance where attitude will be stored.
     */
    public static void getAttitude(
            final NEDPosition position,
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY, final double angularRateZ,
            final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getHeight(), fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param latitude   device latitude expressed in radians (rad).
     * @param height     device height expressed in meters (m).
     * @param kinematics body kinematics containing measured
     *                   body specific force and angular rate.
     * @param result     instance where attitude will be stored.
     */
    public static void getAttitude(
            final double latitude, final double height, final BodyKinematics kinematics,
            final CoordinateTransformation result) {
        getAttitude(latitude, height, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(), kinematics.getAngularRateZ(), result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param position   device position expressed in NED frame.
     * @param kinematics body kinematics containing measured
     *                   body specific force and angular rate.
     * @param result     instance where attitude will be stored.
     */
    public static void getAttitude(
            final NEDPosition position, final BodyKinematics kinematics, final CoordinateTransformation result) {
        getAttitude(position.getLatitude(), position.getHeight(), kinematics, result);
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param latitude     device latitude expressed in radians (rad).
     * @param height       device height expressed in meters (m).
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return estimated attitude.
     */
    public static CoordinateTransformation getAttitude(
            final double latitude, final double height,
            final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY, final double angularRateZ) {
        final var result = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        getAttitude(latitude, height, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param position     device position expressed in NED frame.
     * @param fx           x-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           y-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           z-coordinate of measured body specific force
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX x-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateY y-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @param angularRateZ z-coordinate of body angular rate expressed in
     *                     radians per second (rad/s).
     * @return estimated attitude.
     */
    public static CoordinateTransformation getAttitude(
            final NEDPosition position, final double fx, final double fy, final double fz,
            final double angularRateX, final double angularRateY, final double angularRateZ) {
        final var result = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        getAttitude(position, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param latitude   device latitude expressed in radians (rad).
     * @param height     device height expressed in meters (m).
     * @param kinematics body kinematics containing measured
     *                   body specific force and angular rate.
     * @return estimated attitude.
     */
    public static CoordinateTransformation getAttitude(
            final double latitude, final double height, final BodyKinematics kinematics) {
        final var result = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        getAttitude(latitude, height, kinematics, result);
        return result;
    }

    /**
     * Gets body attitude expressed in the local navigation frame.
     *
     * @param position   device position expressed in NED frame.
     * @param kinematics body kinematics containing measured
     *                   body specific force and angular rate.
     * @return estimated attitude.
     */
    public static CoordinateTransformation getAttitude(final NEDPosition position, final BodyKinematics kinematics) {
        final var result = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        getAttitude(position, kinematics, result);
        return result;
    }

    /**
     * Gets partial body attitude where only roll and pitch angles
     * are reliable.
     *
     * @param latitude device latitude expressed in radians (rad).
     * @param height   device height expressed in meters (m).
     * @param fx       x-coordinate of measured body specific force
     *                 expressed in meters per squared second (m/s^2).
     * @param fy       y-coordinate of measured body specific force
     *                 expressed in meters per squared second (m/s^2).
     * @param fz       z-coordinate of measured body specific force
     *                 expressed in meters per squared second (m/s^2).
     * @param result   instance where partial body attitude will be stored.
     */
    static void getPartialAttitude(
            final double latitude, final double height, final double fx, final double fy, final double fz,
            final CoordinateTransformation result) {

        try {
            // get normalized vector from measured specific force, which
            // mainly contains sensed gravity in the local navigation frame
            // when device is static (Coriolis force is neglected in this
            // implementation).

            // obtain normalized specific force in local navigation coordinates
            final var normF = new double[]{fx, fy, fz};
            ArrayUtils.normalize(normF);

            // obtain gravity in NED coordinates (locally equivalent to
            // the one in local navigation frame).
            // Because Earth is not fully spherical, normalized vector won't
            // be (0, 0, 1), because there will always be a small north
            // gravity component.
            final var nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);

            final var normG = nedGravity.asArray();

            // ensure that down coordinate points towards Earth center, just
            // like sensed specific force
            ArrayUtils.multiplyByScalar(normG, -1.0, normG);

            ArrayUtils.normalize(normG);

            // compute angle between both normalized vectors using dot product
            // cos(alpha) = normF' * normG
            final var cosAlpha = ArrayUtils.dotProduct(normF, normG);

            // compute vector perpendicular to both normF and normG which will
            // be the rotation axis
            final var skew = Utils.skewMatrix(normG);
            final var tmp1 = Matrix.newFromArray(normF);
            final var tmp2 = skew.multiplyAndReturnNew(tmp1);

            final var sinAlpha = Utils.normF(tmp2);

            final var axis = tmp2.toArray();
            ArrayUtils.normalize(axis);

            final var alpha = Math.atan2(sinAlpha, cosAlpha);

            final var q = new Quaternion(axis, alpha);

            // set rotation (yaw angle will be arbitrary and will need
            // to be fixed later on)
            result.setSourceType(FrameType.LOCAL_NAVIGATION_FRAME);
            result.setDestinationType(FrameType.BODY_FRAME);
            result.fromRotation(q);

        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }
}
