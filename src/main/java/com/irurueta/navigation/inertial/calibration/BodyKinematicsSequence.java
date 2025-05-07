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

import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Contains a collection of items containing body kinematics
 * measurements ordered by the timestamp when the measurement was made.
 * Measurements within a sequence will be made while the device is
 * being moved.
 * Samples between sequences will be ignored because it will be assumed
 * that the device will be static.
 * Hence, during static periods, only the mean accelerations will be measured.
 * The mean accelerations during static periods will approximately match the
 * gravity versor expressed in body coordinates.
 *
 * @param <T> a type of {@link TimedBodyKinematics}.
 */
public class BodyKinematicsSequence<T extends TimedBodyKinematics> implements Serializable, Cloneable {

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * List of items.
     * If items are provided unsorted, they are reordered by timestamp on
     * getter method.
     */
    private ArrayList<T> items;

    /**
     * Contains sorted list of items.
     * This list is kept for performance reasons to reduce the amount of
     * required sorting.
     */
    private ArrayList<T> sortedItems;

    /**
     * X-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double beforeMeanFx;

    /**
     * Y-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double beforeMeanFy;

    /**
     * Z-coordinate of mean specific force during the static period happening
     * right before this sequence was measured. Expressed in meters per
     * squared second (m/s^2).
     */
    private double beforeMeanFz;

    /**
     * X-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double afterMeanFx;

    /**
     * Y-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double afterMeanFy;

    /**
     * Z-coordinate of mean specific force during the static period happening
     * right after this sequence was measured. Expressed in meters per squared
     * second (m/s^2).
     */
    private double afterMeanFz;

    /**
     * Constructor.
     */
    public BodyKinematicsSequence() {
    }

    /**
     * Constructor.
     *
     * @param items list of items containing body kinematics to be kept into
     *              this sequence.
     */
    public BodyKinematicsSequence(final List<T> items) {
        setItems(items);
    }

    /**
     * Constructor.
     *
     * @param beforeMeanFx x-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFy y-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFz z-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFx  x-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFy  y-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFz  z-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     */
    public BodyKinematicsSequence(
            final double beforeMeanFx, final double beforeMeanFy, final double beforeMeanFz,
            final double afterMeanFx, final double afterMeanFy, final double afterMeanFz) {
        setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);
        setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);
    }

    /**
     * Constructor.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceX  x-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceY  y-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceZ  z-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     */
    public BodyKinematicsSequence(
            final Acceleration beforeMeanSpecificForceX,
            final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ,
            final Acceleration afterMeanSpecificForceX,
            final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX, beforeMeanSpecificForceY, beforeMeanSpecificForceZ);
        setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX, afterMeanSpecificForceY, afterMeanSpecificForceZ);
    }

    /**
     * @param items        list of items containing body kinematics to be kept into
     *                     this sequence.
     * @param beforeMeanFx x-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFy y-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param beforeMeanFz z-coordinate of mean specific force during the static
     *                     period happening right before this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFx  x-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFy  y-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     * @param afterMeanFz  z-coordinate of mean specific force during the static
     *                     period happening right after this sequence was measured.
     *                     Expressed in meters per squared second (m/s^2).
     */
    public BodyKinematicsSequence(
            final List<T> items, final double beforeMeanFx, final double beforeMeanFy, final double beforeMeanFz,
            final double afterMeanFx, final double afterMeanFy, final double afterMeanFz) {
        this(items);
        setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);
        setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);
    }

    /**
     * Constructor.
     *
     * @param items                    list of items containing body kinematics to be kept into
     *                                 this sequence.
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force during
     *                                 the static period happening right before this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceX  x-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceY  y-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     * @param afterMeanSpecificForceZ  z-coordinate of mean specific force during
     *                                 the static period happening right after this
     *                                 sequence was measured.
     */
    public BodyKinematicsSequence(
            final List<T> items,
            final Acceleration beforeMeanSpecificForceX,
            final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ,
            final Acceleration afterMeanSpecificForceX,
            final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        this(items);
        setBeforeMeanSpecificForceCoordinates(
                beforeMeanSpecificForceX, beforeMeanSpecificForceY, beforeMeanSpecificForceZ);
        setAfterMeanSpecificForceCoordinates(
                afterMeanSpecificForceX, afterMeanSpecificForceY, afterMeanSpecificForceZ);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyKinematicsSequence(final BodyKinematicsSequence<T> input) {
        copyFrom(input);
    }

    /**
     * Gets items in this sequence ordered by ascending timestamp.
     *
     * @param result instance where sorted items will be stored.
     * @return true if sorted items could be retrieved, false if no items
     * are available.
     */
    public boolean getSortedItems(final List<T> result) {
        // already sorted items are available.
        if (sortedItems != null) {
            result.clear();
            result.addAll(sortedItems);
            return true;
        }

        if (items != null) {
            sortedItems = new ArrayList<>(items);
            sortedItems.sort((o1, o2) -> {
                final var t1 = o1.getTimestampSeconds();
                final var t2 = o2.getTimestampSeconds();
                return Double.compare(t1, t2);
            });

            result.clear();
            result.addAll(sortedItems);
            return true;
        }

        return false;
    }

    /**
     * Gets items in this sequence ordered by ascending timestamp.
     *
     * @return a new list containing sorted items or null if sorted items
     * could not be retrieved..
     */
    public List<T> getSortedItems() {
        final var result = new ArrayList<T>();
        if (getSortedItems(result)) {
            return result;
        } else {
            return null;
        }
    }

    /**
     * Sets list of items containing body kinematics to be kept into this
     * sequence.
     *
     * @param items items to be kept.
     */
    public void setItems(final List<T> items) {
        if (items instanceof ArrayList) {
            this.items = (ArrayList<T>) items;
        } else {
            this.items = new ArrayList<>(items);
        }
        sortedItems = null;
    }

    /**
     * Gets number of items in this sequence.
     *
     * @return number of items in this sequence.
     */
    public int getItemsCount() {
        return items != null ? items.size() : 0;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return x-coordinate of mean specific force.
     */
    public double getBeforeMeanFx() {
        return beforeMeanFx;
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFx x-coordinate of mean specific force.
     */
    public void setBeforeMeanFx(final double beforeMeanFx) {
        this.beforeMeanFx = beforeMeanFx;
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return y-coordinate of mean specific force.
     */
    public double getBeforeMeanFy() {
        return beforeMeanFy;
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFy y-coordinate of mean specific force.
     */
    public void setBeforeMeanFy(final double beforeMeanFy) {
        this.beforeMeanFy = beforeMeanFy;
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return z-coordinate of mean specific force.
     */
    public double getBeforeMeanFz() {
        return beforeMeanFz;
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFz z-coordinate of mean specific force.
     */
    public void setBeforeMeanFz(final double beforeMeanFz) {
        this.beforeMeanFz = beforeMeanFz;
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right before this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param beforeMeanFx x-coordinate of mean specific force.
     * @param beforeMeanFy y-coordinate of mean specific force.
     * @param beforeMeanFz z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceCoordinates(
            final double beforeMeanFx, final double beforeMeanFy, final double beforeMeanFz) {
        this.beforeMeanFx = beforeMeanFx;
        this.beforeMeanFy = beforeMeanFy;
        this.beforeMeanFz = beforeMeanFz;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result x-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceX(final Acceleration result) {
        result.setValue(beforeMeanFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return x-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceX() {
        return new Acceleration(beforeMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceX(final Acceleration beforeMeanSpecificForceX) {
        beforeMeanFx = convertAcceleration(beforeMeanSpecificForceX);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result y-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceY(final Acceleration result) {
        result.setValue(beforeMeanFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return y-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceY() {
        return new Acceleration(beforeMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceY(final Acceleration beforeMeanSpecificForceY) {
        beforeMeanFy = convertAcceleration(beforeMeanSpecificForceY);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param result z-coordinate of mean specific force.
     */
    public void getBeforeMeanSpecificForceZ(final Acceleration result) {
        result.setValue(beforeMeanFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @return z-coordinate of mean specific force.
     */
    public Acceleration getBeforeMeanSpecificForceZ() {
        return new Acceleration(beforeMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceZ(final Acceleration beforeMeanSpecificForceZ) {
        beforeMeanFz = convertAcceleration(beforeMeanSpecificForceZ);
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right before this sequence was measured.
     *
     * @param beforeMeanSpecificForceX x-coordinate of mean specific force.
     * @param beforeMeanSpecificForceY y-coordinate of mean specific force.
     * @param beforeMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setBeforeMeanSpecificForceCoordinates(
            final Acceleration beforeMeanSpecificForceX, final Acceleration beforeMeanSpecificForceY,
            final Acceleration beforeMeanSpecificForceZ) {
        setBeforeMeanSpecificForceX(beforeMeanSpecificForceX);
        setBeforeMeanSpecificForceY(beforeMeanSpecificForceY);
        setBeforeMeanSpecificForceZ(beforeMeanSpecificForceZ);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return x-coordinate of mean specific force.
     */
    public double getAfterMeanFx() {
        return afterMeanFx;
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFx x-coordinate of mean specific force.
     */
    public void setAfterMeanFx(final double afterMeanFx) {
        this.afterMeanFx = afterMeanFx;
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return y-coordinate of mean specific force.
     */
    public double getAfterMeanFy() {
        return afterMeanFy;
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFy y-coordinate of mean specific force.
     */
    public void setAfterMeanFy(final double afterMeanFy) {
        this.afterMeanFy = afterMeanFy;
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @return z-coordinate of mean specific force.
     */
    public double getAfterMeanFz() {
        return afterMeanFz;
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFz z-coordinate of mean specific force.
     */
    public void setAfterMeanFz(final double afterMeanFz) {
        this.afterMeanFz = afterMeanFz;
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right after this sequence was measured. Expressed in
     * meters per squared second (m/s^2).
     *
     * @param afterMeanFx x-coordinate of mean specific force.
     * @param afterMeanFy y-coordinate of mean specific force.
     * @param afterMeanFz z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceCoordinates(
            final double afterMeanFx, final double afterMeanFy, final double afterMeanFz) {
        this.afterMeanFx = afterMeanFx;
        this.afterMeanFy = afterMeanFy;
        this.afterMeanFz = afterMeanFz;
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result x-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceX(final Acceleration result) {
        result.setValue(afterMeanFx);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return x-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceX() {
        return new Acceleration(afterMeanFx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceX x-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceX(final Acceleration afterMeanSpecificForceX) {
        afterMeanFx = convertAcceleration(afterMeanSpecificForceX);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result y-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceY(final Acceleration result) {
        result.setValue(afterMeanFy);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return y-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceY() {
        return new Acceleration(afterMeanFy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceY y-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceY(final Acceleration afterMeanSpecificForceY) {
        afterMeanFy = convertAcceleration(afterMeanSpecificForceY);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param result z-coordinate of mean specific force.
     */
    public void getAfterMeanSpecificForceZ(final Acceleration result) {
        result.setValue(afterMeanFz);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @return z-coordinate of mean specific force.
     */
    public Acceleration getAfterMeanSpecificForceZ() {
        return new Acceleration(afterMeanFz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceZ(final Acceleration afterMeanSpecificForceZ) {
        afterMeanFz = convertAcceleration(afterMeanSpecificForceZ);
    }

    /**
     * Sets coordinates of mean specific force during the static period
     * happening right after this sequence was measured.
     *
     * @param afterMeanSpecificForceX x-coordinate of mean specific force.
     * @param afterMeanSpecificForceY y-coordinate of mean specific force.
     * @param afterMeanSpecificForceZ z-coordinate of mean specific force.
     */
    public void setAfterMeanSpecificForceCoordinates(
            final Acceleration afterMeanSpecificForceX, final Acceleration afterMeanSpecificForceY,
            final Acceleration afterMeanSpecificForceZ) {
        setAfterMeanSpecificForceX(afterMeanSpecificForceX);
        setAfterMeanSpecificForceY(afterMeanSpecificForceY);
        setAfterMeanSpecificForceZ(afterMeanSpecificForceZ);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyKinematicsSequence<T> input) {
        if (input.items != null) {
            items = cloneList(input.items);
        } else {
            items = null;
        }
        if (input.sortedItems != null) {
            sortedItems = cloneList(input.sortedItems);
        } else {
            sortedItems = null;
        }

        beforeMeanFx = input.beforeMeanFx;
        beforeMeanFy = input.beforeMeanFy;
        beforeMeanFz = input.beforeMeanFz;

        afterMeanFx = input.afterMeanFx;
        afterMeanFy = input.afterMeanFy;
        afterMeanFz = input.afterMeanFz;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyKinematicsSequence<T> output) {
        output.copyFrom(this);
    }

    /***
     * Checks if provided instance is a BodyKinematicsSequence2 having exactly
     * the same contents as this instance.
     *
     * @param o object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }

        final var that = (BodyKinematicsSequence<?>) o;
        return Double.compare(that.beforeMeanFx, beforeMeanFx) == 0 &&
                Double.compare(that.beforeMeanFy, beforeMeanFy) == 0 &&
                Double.compare(that.beforeMeanFz, beforeMeanFz) == 0 &&
                Double.compare(that.afterMeanFx, afterMeanFx) == 0 &&
                Double.compare(that.afterMeanFy, afterMeanFy) == 0 &&
                Double.compare(that.afterMeanFz, afterMeanFz) == 0 &&
                Objects.equals(items, that.items) &&
                Objects.equals(sortedItems, that.sortedItems);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(items, sortedItems, beforeMeanFx, beforeMeanFy, beforeMeanFz,
                afterMeanFx, afterMeanFy, afterMeanFz);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        //noinspection unchecked
        final var result = (BodyKinematicsSequence<T>) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Clones a list of {@link TimedBodyKinematics}.
     *
     * @param list list to be cloned.
     * @return cloned list.
     */
    @SuppressWarnings("unchecked")
    private ArrayList<T> cloneList(final List<T> list) {
        // constructor with list only creates a new list containing the
        // same instances as the original list.
        // ArrayList is publicly Cloneable, so we clone it to get copies
        // of the elements contained within.

        final var result = new ArrayList<T>();
        for (final var item : list) {
            if (item instanceof StandardDeviationTimedBodyKinematics) {
                final var newItem = new StandardDeviationTimedBodyKinematics();
                newItem.copyFrom(item);
                result.add((T) newItem);
            } else {
                final var newItem = new TimedBodyKinematics(item);
                result.add((T) newItem);
            }
        }

        return result;
    }

    /**
     * Converts an acceleration instance into its corresponding value
     * expressed in meters per squared second.
     *
     * @param acceleration an acceleration to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }
}
