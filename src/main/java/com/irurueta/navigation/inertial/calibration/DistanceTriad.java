/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;

import java.io.Serial;

/**
 * Contains a triad of distance (aka position) measurements.
 */
public class DistanceTriad extends Triad<DistanceUnit, Distance, DistanceTriad> implements Cloneable {

    /**
     * Default distance unit.
     */
    public static final DistanceUnit DEFAULT_UNIT = DistanceUnit.METER;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Constructor.
     */
    public DistanceTriad() {
        this(DEFAULT_UNIT);
    }

    /**
     * Constructor.
     *
     * @param unit distance unit for stored values.
     */
    public DistanceTriad(final DistanceUnit unit) {
        super(unit);
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in default unit.
     * @param valueY y-coordinate of measurement value expressed in default unit.
     * @param valueZ z-coordinate of measurement value expressed in default unit.
     */
    public DistanceTriad(final double valueX, final double valueY, final double valueZ) {
        this(DEFAULT_UNIT, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param unit   acceleration unit for stored values.
     * @param valueX x-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueY y-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueZ z-coordinate of measurement value expressed in
     *               provided unit.
     */
    public DistanceTriad(
            final DistanceUnit unit, final double valueX, final double valueY, final double valueZ) {
        super(unit, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param measurementX x-coordinate of measurement.
     * @param measurementY y-coordinate of measurement.
     * @param measurementZ z-coordinate of measurement.
     */
    public DistanceTriad(
            final Distance measurementX, final Distance measurementY, final Distance measurementZ) {
        super(DEFAULT_UNIT);
        setMeasurementCoordinates(measurementX, measurementY, measurementZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    public DistanceTriad(final DistanceTriad other) {
        super(other);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    @Override
    public Distance getMeasurementX() {
        return new Distance(getValueX(), getUnit());
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementX(final Distance result) {
        result.setValue(getValueX());
        result.setUnit(getUnit());
    }

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    @Override
    public void setMeasurementX(final Distance measurementX) {
        setValueX(DistanceConverter.convert(measurementX.getValue(), measurementX.getUnit(),
                getUnit()).doubleValue());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    @Override
    public Distance getMeasurementY() {
        return new Distance(getValueY(), getUnit());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementY(final Distance result) {
        result.setValue(getValueY());
        result.setUnit(getUnit());
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    @Override
    public void setMeasurementY(final Distance measurementY) {
        setValueY(DistanceConverter.convert(measurementY.getValue(), measurementY.getUnit(),
                getUnit()).doubleValue());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    @Override
    public Distance getMeasurementZ() {
        return new Distance(getValueZ(), getUnit());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementZ(final Distance result) {
        result.setValue(getValueZ());
        result.setUnit(getUnit());
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementZ(final Distance measurementZ) {
        setValueZ(DistanceConverter.convert(measurementZ.getValue(), measurementZ.getUnit(),
                getUnit()).doubleValue());
    }

    /**
     * Sets measurement coordinates.
     *
     * @param measurementX x coordinate of measurement value.
     * @param measurementY y coordinate of measurement value.
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementCoordinates(
            final Distance measurementX, final Distance measurementY, final Distance measurementZ) {
        setMeasurementX(measurementX);
        setMeasurementY(measurementY);
        setMeasurementZ(measurementZ);
    }

    /**
     * Gets norm as an acceleration.
     *
     * @return acceleration containing triad norm.
     */
    @Override
    public Distance getMeasurementNorm() {
        return new Distance(getNorm(), getUnit());
    }

    /**
     * Creates and returns a new instance having exactly the same contents
     * as this instance.
     *
     * @return a copy of this instance.
     */
    @Override
    public DistanceTriad copy() {
        final var result = new DistanceTriad();
        result.copyFrom(this);
        return result;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (DistanceTriad) super.clone();
        copyTo(result);
        return result;
    }
}
