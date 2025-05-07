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
import com.irurueta.units.Measurement;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a triad of measurement data.
 *
 * @param <U> type of unit.
 * @param <T> a type of measurement.
 */
public abstract class Triad<U extends Enum<?>, T extends Measurement<U>> implements Serializable {

    /**
     * Number of components of measurements.
     */
    public static final int COMPONENTS = 3;

    /**
     * Contains x coordinate of measurement value.
     */
    private double valueX;

    /**
     * Contains y coordinate of measurement value.
     */
    private double valueY;

    /**
     * Contains z coordinate of measurement value.
     */
    private double valueZ;

    /**
     * Contains unit of measurement.
     */
    private U unit;

    /**
     * Constructor.
     *
     * @param unit unit of measurement.
     * @throws IllegalArgumentException if provided unit is null.
     */
    protected Triad(final U unit) {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        this.unit = unit;
    }

    /**
     * Constructor.
     *
     * @param unit   unit of measurement.
     * @param valueX value of x-coordinate.
     * @param valueY value of y-coordinate.
     * @param valueZ value of z-coordinate.
     * @throws IllegalArgumentException if provided unit is null.
     */
    protected Triad(final U unit, final double valueX, final double valueY, final double valueZ) {
        this(unit);
        setValueCoordinates(valueX, valueY, valueZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    protected Triad(final Triad<U, T> other) {
        copyFrom(other);
    }

    /**
     * Gets x coordinate of measurement value expressed in current unit.
     *
     * @return x coordinate of measurement value.
     */
    public double getValueX() {
        return valueX;
    }

    /**
     * Sets x coordinate of measurement value using current unit.
     *
     * @param valueX x coordinate of measurement value.
     */
    public void setValueX(final double valueX) {
        this.valueX = valueX;
    }

    /**
     * Gets y coordinate of measurement value expressed in current unit.
     *
     * @return y coordinate of measurement value.
     */
    public double getValueY() {
        return valueY;
    }

    /**
     * Sets y coordinate of measurement value using current unit.
     *
     * @param valueY y coordinate of measurement value.
     */
    public void setValueY(final double valueY) {
        this.valueY = valueY;
    }

    /**
     * Gets z coordinate of measurement value expressed in current unit.
     *
     * @return z coordinate of measurement value.
     */
    public double getValueZ() {
        return valueZ;
    }

    /**
     * Sets z coordinate of measurement value using current unit.
     *
     * @param valueZ z coordinate of measurement value.
     */
    public void setValueZ(final double valueZ) {
        this.valueZ = valueZ;
    }

    /**
     * Sets coordinates of measurement using current unit.
     *
     * @param valueX x coordinate of measurement.
     * @param valueY y coordinate of measurement.
     * @param valueZ z coordinate of measurement.
     */
    public void setValueCoordinates(final double valueX, final double valueY, final double valueZ) {
        this.valueX = valueX;
        this.valueY = valueY;
        this.valueZ = valueZ;
    }

    /**
     * Gets unit of measurement.
     *
     * @return unit of measurement.
     */
    public U getUnit() {
        return unit;
    }

    /**
     * Sets unit of measurement.
     *
     * @param unit unit of measurement.
     * @throws IllegalArgumentException if provided value is null.
     */
    public void setUnit(final U unit) {
        if (unit == null) {
            throw new IllegalArgumentException();
        }
        this.unit = unit;
    }

    /**
     * Sets value coordinates and unit.
     *
     * @param valueX x coordinate of measurement.
     * @param valueY y coordinate of measurement.
     * @param valueZ z coordinate of measurement.
     * @param unit   unit of measurement.
     * @throws IllegalArgumentException if provided unit is null.
     */
    public void setValueCoordinatesAndUnit(
            final double valueX, final double valueY, final double valueZ, final U unit) {
        setValueCoordinates(valueX, valueY, valueZ);
        setUnit(unit);
    }

    /**
     * Gets measurement values as an array expressed in current unit.
     *
     * @return array containing measurement values.
     */
    public double[] getValuesAsArray() {
        return new double[]{valueX, valueY, valueZ};
    }

    /**
     * Gets measurement values as an array expressed in current unit.
     *
     * @param result instance where result will be stored.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getValuesAsArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = valueX;
        result[1] = valueY;
        result[2] = valueZ;
    }

    /**
     * Sets measurement coordinates from provided array.
     *
     * @param values array to set values from.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void setValueCoordinates(final double[] values) {
        if (values.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        valueX = values[0];
        valueY = values[1];
        valueZ = values[2];
    }

    /**
     * Gets measurement values as a column matrix expressed in current unit.
     *
     * @return matrix containing measurement values.
     */
    public Matrix getValuesAsMatrix() {
        Matrix result = null;
        try {
            result = new Matrix(COMPONENTS, 1);
            getValuesAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        return result;
    }

    /**
     * Gets measurement values as a column matrix expressed in current unit.
     *
     * @param result instance where result will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getValuesAsMatrix(final Matrix result) {
        if (result.getRows() != COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, valueX);
        result.setElementAtIndex(1, valueY);
        result.setElementAtIndex(2, valueZ);
    }

    /**
     * Sets measurement coordinates from provided column matrix.
     *
     * @param values matrix to set values from.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setValueCoordinates(final Matrix values) {
        if (values.getRows() != COMPONENTS || values.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        valueX = values.getElementAtIndex(0);
        valueY = values.getElementAtIndex(1);
        valueZ = values.getElementAtIndex(2);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    public abstract T getMeasurementX();

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    public abstract void getMeasurementX(final T result);

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    public abstract void setMeasurementX(final T measurementX);

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    public abstract T getMeasurementY();

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    public abstract void getMeasurementY(final T result);

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    public abstract void setMeasurementY(final T measurementY);

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    public abstract T getMeasurementZ();

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    public abstract void getMeasurementZ(final T result);

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    public abstract void setMeasurementZ(final T measurementZ);

    /**
     * Sets measurement coordinates.
     *
     * @param measurementX x coordinate of measurement value.
     * @param measurementY y coordinate of measurement value.
     * @param measurementZ z coordinate of measurement value.
     */
    public abstract void setMeasurementCoordinates(
            final T measurementX, final T measurementY, final T measurementZ);

    /**
     * Gets squared norm expressed in squared current unit.
     *
     * @return squared norm for triad values.
     */
    public double getSqrNorm() {
        return valueX * valueX + valueY * valueY + valueZ * valueZ;
    }

    /**
     * Gets norm expressed in current unit.
     *
     * @return norm for triad values.
     */
    public double getNorm() {
        return Math.sqrt(getSqrNorm());
    }

    /**
     * Gets norm as a measurement.
     *
     * @return norm as a measurement.
     */
    public abstract T getMeasurementNorm();

    /**
     * Gets norm as a measurement.
     *
     * @param result instance where norm value will be stored.
     */
    public void getMeasurementNorm(final T result) {
        result.setValue(getNorm());
        result.setUnit(getUnit());
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final Triad<U, T> output) {
        output.copyFrom(this);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final Triad<U, T> input) {
        valueX = input.valueX;
        valueY = input.valueY;
        valueZ = input.valueZ;
        unit = input.unit;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fas classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(valueX, valueY, valueZ, unit);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final Triad<U, T> other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final Triad<U, T> other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(valueX - other.valueX) <= threshold && Math.abs(valueY - other.valueY) <= threshold
                && Math.abs(valueZ - other.valueZ) <= threshold && Objects.equals(unit, other.unit);
    }


    /**
     * Checks if provided object is a Triad instance having exactly the same contents
     * as this instance.
     *
     * @param o object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }

        //noinspection unchecked
        final var triad = (Triad<U, T>) o;
        return equals(triad);
    }
}
