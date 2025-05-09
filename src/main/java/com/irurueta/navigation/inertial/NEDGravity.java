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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Contains acceleration due to gravity resolved about NED frame.
 */
public class NEDGravity implements Serializable, Cloneable {

    /**
     * Acceleration due to gravity through east-axis of NED frame expressed in meters per squared second (m/s^2).
     */
    public static final double GRAVITY_EAST = 0.0;

    /**
     * Number of components.
     */
    public static final int COMPONENTS = 3;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Acceleration due to gravity through north-axis of NED frame and expressed in meters per squared second (m/s^2).
     */
    private double gn;

    /**
     * Acceleration due to gravity through down-axis of NED frame and expressed in meters per squared second (m/s^2).
     */
    private double gd;

    /**
     * Constructor.
     */
    public NEDGravity() {
    }

    /**
     * Constructor.
     *
     * @param gn acceleration due to gravity through north-axis of NED frame and expressed in meters per
     *           squared second (m/s^2).
     * @param gd acceleration due to gravity through down-axis of NED frame and expressed in meters per
     *           squared second (m/s^2).
     */
    public NEDGravity(final double gn, final double gd) {
        setCoordinates(gn, gd);
    }

    /**
     * Constructor.
     *
     * @param gn acceleration due to gravity through north-axis of NED frame.
     * @param gd acceleration due to gravity through down-axis of NED frame.
     */
    public NEDGravity(final Acceleration gn, final Acceleration gd) {
        setCoordinates(gn, gd);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public NEDGravity(final NEDGravity input) {
        copyFrom(input);
    }

    /**
     * Gets acceleration due to gravity through north-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through north-axis of NED frame expressed in meters per squared second
     * (m/s^2).
     */
    public double getGn() {
        return gn;
    }

    /**
     * Sets acceleration due to gravity through north-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @param gn acceleration due to gravity through north-axis of NED frame expressed in meters per squared second
     *           (m/s^2).
     */
    public void setGn(final double gn) {
        this.gn = gn;
    }

    /**
     * Gets acceleration due to gravity through east-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through east-axis of NED frame expressed in meters per squared second
     * (m/s^2).
     */
    public double getGe() {
        return GRAVITY_EAST;
    }

    /**
     * Gets acceleration due to gravity through down-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through down-axis of NED frame expressed in meters per squared second
     * (m/s^2).
     */
    public double getGd() {
        return gd;
    }

    /**
     * Sets acceleration due to gravity through down-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @param gd acceleration due to gravity through down-axis of NED frame expressed in meters per squared second
     *           (m/s^2).
     */
    public void setGd(final double gd) {
        this.gd = gd;
    }

    /**
     * Sets gravity coordinates resolved about NED frame and expressed in meters per squared second (m/s^2).
     *
     * @param gn acceleration due to gravity through north-axis of NED frame.
     * @param gd acceleration due to gravity through down-axis of NED frame.
     */
    public void setCoordinates(final double gn, final double gd) {
        this.gn = gn;
        this.gd = gd;
    }

    /**
     * Gets acceleration due to gravity through north-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @param result instance where acceleration due to gravity through NED north-axis will be stored.
     */
    public void getGnAsAcceleration(final Acceleration result) {
        result.setValue(gn);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through north-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through NED north-axis.
     */
    public Acceleration getGnAsAcceleration() {
        return new Acceleration(gn, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through north-axis of NED frame.
     *
     * @param gravityN acceleration due to gravity through NED north-axis.
     */
    public void setGn(final Acceleration gravityN) {
        gn = AccelerationConverter.convert(gravityN.getValue().doubleValue(), gravityN.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through east-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @param result instance where acceleration due to gravity through NED east-axis will be stored.
     */
    public void getGeAsAcceleration(final Acceleration result) {
        result.setValue(GRAVITY_EAST);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through east-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through NED east-axis.
     */
    public Acceleration getGeAsAcceleration() {
        return new Acceleration(GRAVITY_EAST, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through down-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @param result instance where acceleration due to gravity through NED down-axis will be stored.
     */
    public void getGdAsAcceleration(final Acceleration result) {
        result.setValue(gd);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets acceleration due to gravity through down-axis of NED frame expressed in meters per squared second (m/s^2).
     *
     * @return acceleration due to gravity through NED down-axis.
     */
    public Acceleration getGdAsAcceleration() {
        return new Acceleration(gd, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration due to gravity through down-axis of NED frame.
     *
     * @param gravityD acceleration due to gravity through NED down-axis.
     */
    public void setGd(final Acceleration gravityD) {
        gd = AccelerationConverter.convert(gravityD.getValue().doubleValue(), gravityD.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets gravity coordinates.
     *
     * @param gravityN acceleration due to gravity through north-axis of NED frame.
     * @param gravityD acceleration due to gravity through down-axis of NED frame.
     */
    public void setCoordinates(final Acceleration gravityN, final Acceleration gravityD) {
        setGn(gravityN);
        setGd(gravityD);
    }

    /**
     * Gets gravity norm.
     *
     * @return gravity norm.
     */
    public double getNorm() {
        return Math.sqrt(gn * gn + gd * gd);
    }

    /**
     * Gets gravity norm as an acceleration.
     *
     * @param result instance where result will be stored.
     */
    public void getNormAsAcceleration(final Acceleration result) {
        result.setValue(getNorm());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets gravity norm as an acceleration.
     *
     * @return an acceleration containing gravity norm.
     */
    public Acceleration getNormAsAcceleration() {
        return new Acceleration(getNorm(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final NEDGravity output) {
        output.gn = gn;
        output.gd = gd;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final NEDGravity input) {
        gn = input.gn;
        gd = input.gd;
    }

    /**
     * Gets gravity coordinates as an array.
     *
     * @param result array instance where gravity coordinates will be stored in
     *               n,e,d order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = gn;
        result[1] = GRAVITY_EAST;
        result[2] = gd;
    }

    /**
     * Gets gravity coordinates as an array.
     *
     * @return array containing gravity coordinates in n,e,d order.
     */
    public double[] asArray() {
        final var result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets gravity coordinates as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where gravity coordinates will be stored in
     *               n,e,d order.
     */
    @SuppressWarnings("DuplicatedCode")
    public void asMatrix(final Matrix result) {
        if (result.getColumns() != COMPONENTS || result.getRows() != 1) {
            try {
                result.resize(COMPONENTS, 1);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }

        result.setElementAtIndex(0, gn);
        result.setElementAtIndex(1, GRAVITY_EAST);
        result.setElementAtIndex(2, gd);
    }

    /**
     * Gets gravity coordinates as a column matrix.
     *
     * @return a matrix containing gravity coordinates stored in n,e,d order.
     */
    public Matrix asMatrix() {
        Matrix result;
        try {
            result = new Matrix(COMPONENTS, 1);
            asMatrix(result);
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
        return Objects.hash(gn, gd);
    }

    /**
     * Check if provided object is a GravityNED instance having exactly the same contents
     * as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof NEDGravity)) {
            return false;
        }

        //noinspection PatternVariableCanBeUsed
        final var other = (NEDGravity) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final NEDGravity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between gravity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final NEDGravity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(gn - other.gn) <= threshold && Math.abs(gd - other.gd) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (NEDGravity) super.clone();
        copyTo(result);
        return result;
    }
}
