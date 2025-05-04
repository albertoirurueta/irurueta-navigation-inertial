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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Contains magnetic flux density resolved around body coordinates.
 * Body frame axes are typically defined so that x is the forward axis, pointing in the usual direction
 * of travel, z is the down axis, pointing in the usual direction of gravity, and y is the right axis,
 * completing the orthogonal set.
 */
public class BodyMagneticFluxDensity implements Serializable, Cloneable {

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
     * X component of magnetic flux density expressed in Teslas (T).
     */
    private double bx;

    /**
     * Y component of magnetic flux density expressed in Teslas (T).
     */
    private double by;

    /**
     * Z component of magnetic flux density expressed in Teslas (T).
     */
    private double bz;

    /**
     * Constructor.
     */
    public BodyMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param bx x component of magnetic flux density expressed in Teslas (T).
     * @param by y component of magnetic flux density expressed in Teslas (T).
     * @param bz z component of magnetic flux density expressed in Teslas (T).
     */
    public BodyMagneticFluxDensity(final double bx, final double by, final double bz) {
        setCoordinates(bx, by, bz);
    }

    /**
     * Constructor.
     *
     * @param bx x component of magnetic flux density.
     * @param by y component of magnetic flux density.
     * @param bz z component of magnetic flux density.
     */
    public BodyMagneticFluxDensity(
            final MagneticFluxDensity bx, final MagneticFluxDensity by, final MagneticFluxDensity bz) {
        setCoordinates(bx, by, bz);
    }

    /**
     * Constructor.
     *
     * @param triad triad containing magnetic flux density values.
     */
    public BodyMagneticFluxDensity(final MagneticFluxDensityTriad triad) {
        setCoordinates(triad);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyMagneticFluxDensity(final BodyMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets x component of magnetic flux density expressed in Teslas (T).
     *
     * @return x component of magnetic flux density.
     */
    public double getBx() {
        return bx;
    }

    /**
     * Sets x component of magnetic flux density expressed in Teslas (T).
     *
     * @param bx x component of magnetic flux density.
     */
    public void setBx(final double bx) {
        this.bx = bx;
    }

    /**
     * Gets x component of magnetic flux density.
     *
     * @return x component of magnetic flux density.
     */
    public MagneticFluxDensity getBxAsMagneticFluxDensity() {
        return new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets x component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getBxAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(bx);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets x component of magnetic flux density.
     *
     * @param bx x component of magnetic flux density.
     */
    public void setBx(final MagneticFluxDensity bx) {
        this.bx = convertMagneticFluxDensity(bx);
    }

    /**
     * Gets y component of magnetic flux density expressed in Teslas (T).
     *
     * @return y component of magnetic flux density.
     */
    public double getBy() {
        return by;
    }

    /**
     * Sets y component of magnetic flux density expressed in Teslas (T).
     *
     * @param by y component of magnetic flux density.
     */
    public void setBy(final double by) {
        this.by = by;
    }

    /**
     * Gets y component of magnetic flux density.
     *
     * @return y component of magnetic flux density.
     */
    public MagneticFluxDensity getByAsMagneticFluxDensity() {
        return new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets y component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getByAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(by);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets y component of magnetic flux density.
     *
     * @param by y component of magnetic flux density.
     */
    public void setBy(final MagneticFluxDensity by) {
        this.by = convertMagneticFluxDensity(by);
    }

    /**
     * Gets z component of magnetic flux density expressed in Teslas (T).
     *
     * @return z component of magnetic flux density.
     */
    public double getBz() {
        return bz;
    }

    /**
     * Sets z component of magnetic flux density expressed in Teslas (T).
     *
     * @param bz z component of magnetic flux density.
     */
    public void setBz(final double bz) {
        this.bz = bz;
    }

    /**
     * Gets z component of magnetic flux density.
     *
     * @return z component of magnetic flux density.
     */
    public MagneticFluxDensity getBzAsMagneticFluxDensity() {
        return new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets z component of magnetic flux density.
     *
     * @param result instance where result will be stored.
     */
    public void getBzAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(bz);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets z component of magnetic flux density.
     *
     * @param bz z component of magnetic flux density.
     */
    public void setBz(final MagneticFluxDensity bz) {
        this.bz = convertMagneticFluxDensity(bz);
    }

    /**
     * Sets body coordinates of magnetic flux density expressed in Teslas (T).
     *
     * @param bx x component of magnetic flux density.
     * @param by y component of magnetic flux density.
     * @param bz z component of magnetic flux density.
     */
    public void setCoordinates(final double bx, final double by, final double bz) {
        this.bx = bx;
        this.by = by;
        this.bz = bz;
    }

    /**
     * Sets body coordinates of magnetic flux density.
     *
     * @param bx x component of magnetic flux density.
     * @param by y component of magnetic flux density.
     * @param bz z component of magnetic flux density.
     */
    public void setCoordinates(
            final MagneticFluxDensity bx, final MagneticFluxDensity by, final MagneticFluxDensity bz) {
        setCoordinates(convertMagneticFluxDensity(bx), convertMagneticFluxDensity(by), convertMagneticFluxDensity(bz));
    }

    /**
     * Gets body coordinates of magnetic flux density as a triad.
     *
     * @return body coordinates of magnetic flux density as a triad.
     */
    public MagneticFluxDensityTriad getCoordinatesAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, bx, by, bz);
    }

    /**
     * Gets body coordinates of magnetic flux density as a triad.
     *
     * @param result instance where result will be stored.
     */
    public void getCoordinatesAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(bx, by, bz, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets body coordinates of magnetic flux density.
     *
     * @param triad triad containing body magnetic flux density values.
     */
    public void setCoordinates(final MagneticFluxDensityTriad triad) {
        final var tmpBx = convertMagneticFluxDensity(triad.getValueX(), triad.getUnit());
        final var tmpBy = convertMagneticFluxDensity(triad.getValueY(), triad.getUnit());
        final var tmpBz = convertMagneticFluxDensity(triad.getValueZ(), triad.getUnit());
        setCoordinates(tmpBx, tmpBy, tmpBz);
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm) expressed in
     * Teslas (T).
     *
     * @return magnetic flux density magnitude.
     */
    public double getNorm() {
        return Math.sqrt(bx * bx + by * by + bz * bz);
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm).
     *
     * @return magnetic flux density magnitude.
     */
    public MagneticFluxDensity getNormAsMagneticFluxDensity() {
        return new MagneticFluxDensity(getNorm(), MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets magnetic flux density magnitude (e.g. norm).
     *
     * @param result instance where result will be stored.
     */
    public void getNormAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(getNorm());
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyMagneticFluxDensity output) {
        output.bx = bx;
        output.by = by;
        output.bz = bz;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyMagneticFluxDensity input) {
        bx = input.bx;
        by = input.by;
        bz = input.bz;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @param result array instance where magnetic flux density coordinates
     *               will be stored in x,y,z order.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void asArray(final double[] result) {
        if (result.length != COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = bx;
        result[1] = by;
        result[2] = bz;
    }

    /**
     * Gets magnetic flux density as an array.
     *
     * @return array containing magnetic flux density coordinates in x,y,z
     * order.
     */
    public double[] asArray() {
        final var result = new double[COMPONENTS];
        asArray(result);
        return result;
    }

    /**
     * Gets magnetic flux density as a column matrix.
     * If provided matrix does not have size 3x1, it will be resized.
     *
     * @param result matrix instance where magnetic flux density coordinates
     *               will be stored in x,y,z order.
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

        result.setElementAtIndex(0, bx);
        result.setElementAtIndex(1, by);
        result.setElementAtIndex(2, bz);
    }

    /**
     * Gets magnetic flux density as a column matrix.
     *
     * @return a matrix containing magnetic flux density coordinates stored
     * in x,y,z order.
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
        return Objects.hash(bx, by, bz);
    }

    /**
     * Check if provided object is a BodyMagneticFluxDensity instance having
     * exactly the same contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false
     * otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof BodyMagneticFluxDensity)) {
            return false;
        }

        //noinspection PatternVariableCanBeUsed
        final var other = (BodyMagneticFluxDensity) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this
     * instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false
     * otherwise.
     */
    public boolean equals(final BodyMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up
     * to provided threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between gravity coordinates.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final BodyMagneticFluxDensity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(bx - other.bx) <= threshold && Math.abs(by - other.by) <= threshold
                && Math.abs(bz - other.bz) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for same reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (BodyMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Converts magnetic flux density to Teslas.
     *
     * @param b magnetic flux density to be converted.
     * @return converted value.
     */
    private double convertMagneticFluxDensity(final MagneticFluxDensity b) {
        return MagneticFluxDensityConverter.convert(b.getValue().doubleValue(), b.getUnit(),
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Converts magnetic flux density to Teslas.
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted
     * @return converted value.
     */
    private double convertMagneticFluxDensity(final double value, final MagneticFluxDensityUnit unit) {
        return MagneticFluxDensityConverter.convert(value, unit, MagneticFluxDensityUnit.TESLA);
    }
}
