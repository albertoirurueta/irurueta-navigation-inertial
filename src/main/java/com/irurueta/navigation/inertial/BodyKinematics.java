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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Describes the motion of a body based on the specific forces (i.e. specific acceleration) and
 * angular rates applied to it.
 * Body frame axes are typically defined so that x is the forward axis, pointing in the usual direction
 * of travel, z is the down axis, pointing in the usual direction of gravity, and y is the right axis,
 * completing the orthogonal set.
 */
@SuppressWarnings("DuplicatedCode")
public class BodyKinematics implements Serializable, Cloneable {

    /**
     * Number of components of specific force or angular rate.
     */
    public static final int COMPONENTS = 3;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    private double mFx;

    /**
     * Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    private double mFy;

    /**
     * Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     */
    private double mFz;

    /**
     * Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    private double mAngularRateX;

    /**
     * Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    private double mAngularRateY;

    /**
     * Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     */
    private double mAngularRateZ;

    /**
     * Constructor.
     */
    public BodyKinematics() {
    }

    /**
     * Constructor.
     *
     * @param fx Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public BodyKinematics(final double fx, final double fy, final double fz) {
        setSpecificForceCoordinates(fx, fy, fz);
    }

    /**
     * Constructor.
     *
     * @param fx           Specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                     body-frame x-axis, averaged over time interval and expressed in meters per
     *                     squared second (m/s^2).
     * @param fy           Specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                     body-frame y-axis, averaged over time interval and expressed in meters per
     *                     squared second (m/s^2).
     * @param fz           Specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                     body-frame z-axis, averaged over time interval and expressed in meters per
     *                     squared second (m/s^2).
     * @param angularRateX Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about
     *                     body-frame x-axis, averaged over time interval and expressed in radians per
     *                     second (rad/s).
     * @param angularRateY Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about
     *                     body-frame y-axis, averaged over time interval and expressed in radians per
     *                     second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about
     *                     body-frame z-axis, averaged over time interval and expressed in radians per
     *                     second (rad/s).
     */
    public BodyKinematics(final double fx, final double fy, final double fz,
                          final double angularRateX, final double angularRateY, final double angularRateZ) {
        setSpecificForceCoordinates(fx, fy, fz);
        setAngularRateCoordinates(angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame z-axis, averaged over time interval.
     */
    public BodyKinematics(
            final Acceleration specificForceX, final Acceleration specificForceY, final Acceleration specificForceZ) {
        setSpecificForceCoordinates(specificForceX, specificForceY, specificForceZ);
    }

    /**
     * Constructor.
     *
     * @param angularSpeedX Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                      about body-frame x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                      about body-frame y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                      about body-frame z-axis, averaged over time interval.
     */
    public BodyKinematics(
            final AngularSpeed angularSpeedX, final AngularSpeed angularSpeedY, final AngularSpeed angularSpeedZ) {
        setAngularSpeedCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceX Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI, ECEF or NED frame resolved
     *                       along body-frame z-axis, averaged over time interval.
     * @param angularSpeedX  Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                       about body-frame x-axis, averaged over time interval.
     * @param angularSpeedY  Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                       about body-frame y-axis, averaged over time interval.
     * @param angularSpeedZ  Angular speed of body frame with respect ECI, ECEF or NED frame, resolved
     *                       about body-frame z-axis, averaged over time interval.
     */
    public BodyKinematics(
            final Acceleration specificForceX, final Acceleration specificForceY, final Acceleration specificForceZ,
            final AngularSpeed angularSpeedX, final AngularSpeed angularSpeedY, final AngularSpeed angularSpeedZ) {
        setSpecificForceCoordinates(specificForceX, specificForceY, specificForceZ);
        setAngularSpeedCoordinates(angularSpeedX, angularSpeedY, angularSpeedZ);
    }

    /**
     * Constructor.
     *
     * @param specificForceTriad specific force triad.
     * @param angularSpeedTriad  angular speed triad.
     */
    public BodyKinematics(
            final AccelerationTriad specificForceTriad, final AngularSpeedTriad angularSpeedTriad) {
        setSpecificForceTriad(specificForceTriad);
        setAngularRateTriad(angularSpeedTriad);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyKinematics(final BodyKinematics input) {
        copyFrom(input);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis.
     */
    public double getFx() {
        return mFx;
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fx specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis.
     */
    public void setFx(final double fx) {
        mFx = fx;
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis.
     */
    public double getFy() {
        return mFy;
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fy specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis.
     */
    public void setFy(final double fy) {
        mFy = fy;
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @return specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis.
     */
    public double getFz() {
        return mFz;
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fz specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis.
     */
    public void setFz(final double fz) {
        mFz = fz;
    }

    /**
     * Sets specific force coordinates of body frame with respect ECI, ECEF or NED frame resolved along body-frame
     * axes, averaged over time interval and expressed in meters per squared second (m/s^2).
     *
     * @param fx Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fy Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     * @param fz Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis,
     *           averaged over time interval and expressed in meters per squared second (m/s^2).
     */
    public void setSpecificForceCoordinates(final double fx, final double fy, final double fz) {
        mFx = fx;
        mFy = fy;
        mFz = fz;
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *               body-frame x-axis will be stored.
     */
    public void getSpecificForceX(final Acceleration result) {
        result.setValue(mFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI, ECEF or NED frame resolved along
     * body-frame x-axis.
     */
    public Acceleration getSpecificForceX() {
        return new Acceleration(mFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame x-axis, averaged
     * over time interval.
     *
     * @param specificForceX specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                       body-frame x-axis that will be set.
     */
    public void setSpecificForceX(final Acceleration specificForceX) {
        mFx = AccelerationConverter.convert(specificForceX.getValue().doubleValue(), specificForceX.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *               body-frame y-axis will be stored.
     */
    public void getSpecificForceY(final Acceleration result) {
        result.setValue(mFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame y-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI, ECEF or NED frame resolved along
     * body-frame y-axis.
     */
    public Acceleration getSpecificForceY() {
        return new Acceleration(mFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body frame y-axis, averaged
     * over time interval.
     *
     * @param specificForceY specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                       body-frame y-axis that will be set.
     */
    public void setSpecificForceY(final Acceleration specificForceY) {
        mFy = AccelerationConverter.convert(specificForceY.getValue().doubleValue(), specificForceY.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @param result instance where specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *               body-frame z-axis will be stored.
     */
    public void getSpecificForceZ(final Acceleration result) {
        result.setValue(mFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @return a new instance with specific force of body frame with respect ECI, ECEF or NED frame resolved along
     * body-frame z-axis.
     */
    public Acceleration getSpecificForceZ() {
        return new Acceleration(mFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame z-axis, averaged
     * over time interval.
     *
     * @param specificForceZ specific force of body frame with respect ECI, ECEF or NED frame resolved along
     *                       body-frame z-axis that will be set.
     */
    public void setSpecificForceZ(final Acceleration specificForceZ) {
        mFz = AccelerationConverter.convert(specificForceZ.getValue().doubleValue(), specificForceZ.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force coordinates of body frame with respect ECI, ECEF or NED frame resolved along body-frame axes,
     * averaged over time interval.
     *
     * @param specificForceX Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame
     *                       x-axis, averaged over time interval.
     * @param specificForceY Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame
     *                       y-axis, averaged over time interval.
     * @param specificForceZ Specific force of body frame with respect ECI, ECEF or NED frame resolved along body-frame
     *                       z-axis, averaged over time interval.
     */
    public void setSpecificForceCoordinates(
            final Acceleration specificForceX, final Acceleration specificForceY, final Acceleration specificForceZ) {
        setSpecificForceX(specificForceX);
        setSpecificForceY(specificForceY);
        setSpecificForceZ(specificForceZ);
    }

    /**
     * Gets specific force triad of accelerometer measurements.
     *
     * @return specific force triad.
     */
    public AccelerationTriad getSpecificForceTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, mFx, mFy, mFz);
    }

    /**
     * Gets specific force triad of accelerometer measurements.
     *
     * @param result instance where result will be stored.
     */
    public void getSpecificForceTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(mFx, mFy, mFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets specific force triad of accelerometer measurements.
     *
     * @param triad specific force triad.
     */
    public void setSpecificForceTriad(final AccelerationTriad triad) {
        final AccelerationUnit unit = triad.getUnit();
        mFx = AccelerationConverter.convert(triad.getValueX(), unit, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        mFy = AccelerationConverter.convert(triad.getValueY(), unit, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        mFz = AccelerationConverter.convert(triad.getValueZ(), unit, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI, ECEF or NED frame resolved about body-frame x-axis.
     */
    public double getAngularRateX() {
        return mAngularRateX;
    }

    /**
     * Sets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis,
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateX angular rate of body frame with respect ECI, ECEF or NED frame resolved
     *                     about body-frame x-axis.
     */
    public void setAngularRateX(final double angularRateX) {
        mAngularRateX = angularRateX;
    }

    /**
     * Gets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI, ECEF or NED frame resolved about body-frame y-axis.
     */
    public double getAngularRateY() {
        return mAngularRateY;
    }

    /**
     * Sets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateY angular rate of body frame with respect ECI, ECEF or NED frame resolved
     *                     about body-frame y-axis.
     */
    public void setAngularRateY(final double angularRateY) {
        mAngularRateY = angularRateY;
    }

    /**
     * Gets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @return angular rate of body frame with respect ECI, ECEF or NED frame resolved about body-frame z-axis.
     */
    public double getAngularRateZ() {
        return mAngularRateZ;
    }

    /**
     * Sets angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame z-axis, averaged
     * over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateZ angular rate of body frame with respect ECI, ECEF or NED frame resolved
     *                     about body-frame z-axis.
     */
    public void setAngularRateZ(final double angularRateZ) {
        mAngularRateZ = angularRateZ;
    }

    /**
     * Sets angular rate coordinates of body frame with respect ECI, ECEF or NED frame, resolved about body-frame axes,
     * averaged over time interval and expressed in radians per second (rad/s).
     *
     * @param angularRateX Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                     x-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateY Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                     y-axis, averaged over time interval and expressed in radians per second (rad/s).
     * @param angularRateZ Angular rate of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                     z-axis, averaged over time interval and expressed in radians per second (rad/s).
     */
    public void setAngularRateCoordinates(
            final double angularRateX, final double angularRateY, final double angularRateZ) {
        mAngularRateX = angularRateX;
        mAngularRateY = angularRateY;
        mAngularRateZ = angularRateZ;
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     *               body-frame x-axis will be stored.
     */
    public void getAngularSpeedX(final AngularSpeed result) {
        result.setValue(mAngularRateX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     * body-frame X-axis.
     */
    public AngularSpeed getAngularSpeedX() {
        return new AngularSpeed(mAngularRateX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame x-axis, averaged
     * over time interval.
     *
     * @param angularSpeedX angular speed of body frame with respect ECI, ECEF or NED frame resolved about body-frame
     *                      x-axis that will be set.
     */
    public void setAngularSpeedX(final AngularSpeed angularSpeedX) {
        mAngularRateX = AngularSpeedConverter.convert(angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     *               body-frame y-axis will be stored.
     */
    public void getAngularSpeedY(final AngularSpeed result) {
        result.setValue(mAngularRateY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     * body-frame y-axis.
     */
    public AngularSpeed getAngularSpeedY() {
        return new AngularSpeed(mAngularRateY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param angularSpeedY angular speed of body frame with respect ECI, ECEF or NED frame resolved about body-frame
     *                      y-axis that will be set.
     */
    public void setAngularSpeedY(final AngularSpeed angularSpeedY) {
        mAngularRateY = AngularSpeedConverter.convert(angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame z-axis, averaged
     * over time interval.
     *
     * @param result instance where angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     *               body-frame z-axis will be stored.
     */
    public void getAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mAngularRateZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame z-axis, averaged
     * over time interval.
     *
     * @return a new instance of angular speed of body frame with respect ECI, ECEF or NED frame resolved about
     * body-frame z-axis.
     */
    public AngularSpeed getAngularSpeedZ() {
        return new AngularSpeed(mAngularRateZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame y-axis, averaged
     * over time interval.
     *
     * @param angularSpeedZ angular speed of body frame with respect ECI, ECEF or NED frame resolved about body-frame
     *                      z-axis, that will be set.
     */
    public void setAngularSpeedZ(final AngularSpeed angularSpeedZ) {
        mAngularRateZ = AngularSpeedConverter.convert(angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular speed coordinates of body frame with respect ECI, ECEF or NED frame, resolved about body-frame axes,
     * averaged over time interval.
     *
     * @param angularSpeedX Angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                      x-axis, averaged over time interval.
     * @param angularSpeedY Angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                      y-axis, averaged over time interval.
     * @param angularSpeedZ Angular speed of body frame with respect ECI, ECEF or NED frame, resolved about body-frame
     *                      z-axis, averaged over time interval.
     */
    public void setAngularSpeedCoordinates(
            final AngularSpeed angularSpeedX, final AngularSpeed angularSpeedY, final AngularSpeed angularSpeedZ) {
        setAngularSpeedX(angularSpeedX);
        setAngularSpeedY(angularSpeedY);
        setAngularSpeedZ(angularSpeedZ);
    }

    /**
     * Gets angular rate triad of gyroscope measurements.
     *
     * @return angular rate triad.
     */
    public AngularSpeedTriad getAngularRateTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, mAngularRateX, mAngularRateY, mAngularRateZ);
    }

    /**
     * Gets angular rate triad of gyroscope measurements.
     *
     * @param result angular rate triad.
     */
    public void getAngularRateTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(mAngularRateX, mAngularRateY, mAngularRateZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets angular rate triad of gyroscope measurements.
     *
     * @param triad angular rate triad.
     */
    public void setAngularRateTriad(final AngularSpeedTriad triad) {
        final AngularSpeedUnit unit = triad.getUnit();
        mAngularRateX = AngularSpeedConverter.convert(triad.getValueX(), unit, AngularSpeedUnit.RADIANS_PER_SECOND);
        mAngularRateY = AngularSpeedConverter.convert(triad.getValueY(), unit, AngularSpeedUnit.RADIANS_PER_SECOND);
        mAngularRateZ = AngularSpeedConverter.convert(triad.getValueZ(), unit, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of specific force expressed in meters per squared second (m/s^2).
     *
     * @return norm of specific force.
     */
    public double getSpecificForceNorm() {
        return Math.sqrt(mFx * mFx + mFy * mFy + mFz * mFz);
    }

    /**
     * Gets norm of specific force.
     *
     * @param result instance where result will be stored.
     */
    public void getSpecificForceNormAsAcceleration(final Acceleration result) {
        result.setValue(getSpecificForceNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of specific force.
     *
     * @return a new acceleration instance containing norm of specific force.
     */
    public Acceleration getSpecificForceNormAsAcceleration() {
        return new Acceleration(getSpecificForceNorm(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets norm of angular rate expressed in radians per second (rad/s).
     *
     * @return norm of angular rate.
     */
    public double getAngularRateNorm() {
        return Math.sqrt(mAngularRateX * mAngularRateX + mAngularRateY * mAngularRateY
                + mAngularRateZ * mAngularRateZ);
    }

    /**
     * Gets norm of angular rate.
     *
     * @param result instance where norm of angular rate will be stored.
     */
    public void getAngularSpeedNorm(final AngularSpeed result) {
        result.setValue(getAngularRateNorm());
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets norm of angular rate.
     *
     * @return norm of angular rate.
     */
    public AngularSpeed getAngularSpeedNorm() {
        return new AngularSpeed(getAngularRateNorm(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyKinematics output) {
        output.mFx = mFx;
        output.mFy = mFy;
        output.mFz = mFz;
        output.mAngularRateX = mAngularRateX;
        output.mAngularRateY = mAngularRateY;
        output.mAngularRateZ = mAngularRateZ;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyKinematics input) {
        mFx = input.mFx;
        mFy = input.mFy;
        mFz = input.mFz;
        mAngularRateX = input.mAngularRateX;
        mAngularRateY = input.mAngularRateY;
        mAngularRateZ = input.mAngularRateZ;
    }

    /**
     * Gets specific force coordinates expressed in meters per squared second (m/s^2) as an array.
     *
     * @param result array instance where specific force coordinates will be stored in
     *               x, y, z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asSpecificForceArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mFx;
        result[1] = mFy;
        result[2] = mFz;
    }

    /**
     * Gets specific force coordinates expressed in meters per squared second (m/s^2) as an array.
     *
     * @return array containing specific force coordinates in x,y,z order.
     */
    public double[] asSpecificForceArray() {
        final double[] result = new double[COMPONENTS];
        asSpecificForceArray(result);
        return result;
    }

    /**
     * Gets specific force coordinates expressed in meters per squared second (m/s^2) as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where gravity coordinates will be stored in
     *               x,y,z order.
     */
    public void asSpecificForceMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mFx);
        result.setElementAtIndex(1, mFy);
        result.setElementAtIndex(2, mFz);
    }

    /**
     * Gets specific force coordinates expressed in meters per squared second (m/s^2) as a column matrix.
     *
     * @return a matrix containing specific force coordinates stored in x,y,z order.
     */
    public Matrix asSpecificForceMatrix() {
        Matrix result;
        try {
            result = new Matrix(COMPONENTS, 1);
            asSpecificForceMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets angular rate coordinates expressed in radians per second (rad/s) as an array.
     *
     * @param result array instance where angular rate coordinates will be stored in
     *               x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asAngularRateArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mAngularRateX;
        result[1] = mAngularRateY;
        result[2] = mAngularRateZ;
    }

    /**
     * Gets angular rate coordinates expressed in radians per second (rad/s) as an array.
     *
     * @return array containing angular rate coordinates in x,y,z order.
     */
    public double[] asAngularRateArray() {
        final double[] result = new double[COMPONENTS];
        asAngularRateArray(result);
        return result;
    }

    /**
     * Gets angular rate coordinates expressed in radians per second (rad/s) as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where angular rate coordinates will be stored in
     *               x,y,z order.
     */
    public void asAngularRateMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, mAngularRateX);
        result.setElementAtIndex(1, mAngularRateY);
        result.setElementAtIndex(2, mAngularRateZ);
    }

    /**
     * Gets angular rate coordinates expressed in radians per second (rad/s) as a column matrix.
     *
     * @return a matrix containing angular rate coordinates stored in x,y,z order.
     */
    public Matrix asAngularRateMatrix() {
        Matrix result;
        try {
            result = new Matrix(COMPONENTS, 1);
            asAngularRateMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mFx, mFy, mFz, mAngularRateX, mAngularRateY, mAngularRateZ);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final BodyKinematics other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between specific force and angular
     *                  rate coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final BodyKinematics other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mFx - other.mFx) <= threshold && Math.abs(mFy - other.mFy) <= threshold
                && Math.abs(mFz - other.mFz) <= threshold && Math.abs(mAngularRateX - other.mAngularRateX) <= threshold
                && Math.abs(mAngularRateY - other.mAngularRateY) <= threshold
                && Math.abs(mAngularRateZ - other.mAngularRateZ) <= threshold;
    }

    /**
     * Checks if provided object is a BodyKinematics instance having exactly the same contents
     * as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }

        final BodyKinematics other = (BodyKinematics) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final BodyKinematics result = (BodyKinematics) super.clone();
        copyTo(result);
        return result;
    }
}
