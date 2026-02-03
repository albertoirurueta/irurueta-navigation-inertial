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

import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serial;

/**
 * Contains a triad of speed measurements.
 */
public class SpeedTriad extends Triad<SpeedUnit, Speed, SpeedTriad> implements Cloneable {

    /**
     * Default speed unit.
     */
    public static final SpeedUnit DEFAULT_UNIT = SpeedUnit.METERS_PER_SECOND;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Constructor.
     */
    public SpeedTriad() {
        this(DEFAULT_UNIT);
    }

    /**
     * Constructor.
     *
     * @param unit acceleration unit for stored values.
     */
    public SpeedTriad(final SpeedUnit unit) {
        super(unit);
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in default unit.
     * @param valueY y-coordinate of measurement value expressed in default unit.
     * @param valueZ z-coordinate of measurement value expressed in default unit.
     */
    public SpeedTriad(final double valueX, final double valueY, final double valueZ) {
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
    public SpeedTriad(
            final SpeedUnit unit, final double valueX, final double valueY, final double valueZ) {
        super(unit, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param measurementX x-coordinate of measurement.
     * @param measurementY y-coordinate of measurement.
     * @param measurementZ z-coordinate of measurement.
     */
    public SpeedTriad(
            final Speed measurementX, final Speed measurementY, final Speed measurementZ) {
        super(DEFAULT_UNIT);
        setMeasurementCoordinates(measurementX, measurementY, measurementZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    public SpeedTriad(final SpeedTriad other) {
        super(other);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    @Override
    public Speed getMeasurementX() {
        return new Speed(getValueX(), getUnit());
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementX(final Speed result) {
        result.setValue(getValueX());
        result.setUnit(getUnit());
    }

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    @Override
    public void setMeasurementX(final Speed measurementX) {
        setValueX(SpeedConverter.convert(measurementX.getValue(), measurementX.getUnit(),
                getUnit()).doubleValue());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    @Override
    public Speed getMeasurementY() {
        return new Speed(getValueY(), getUnit());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementY(final Speed result) {
        result.setValue(getValueY());
        result.setUnit(getUnit());
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    @Override
    public void setMeasurementY(final Speed measurementY) {
        setValueY(SpeedConverter.convert(measurementY.getValue(), measurementY.getUnit(),
                getUnit()).doubleValue());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    @Override
    public Speed getMeasurementZ() {
        return new Speed(getValueZ(), getUnit());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementZ(final Speed result) {
        result.setValue(getValueZ());
        result.setUnit(getUnit());
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementZ(final Speed measurementZ) {
        setValueZ(SpeedConverter.convert(measurementZ.getValue(), measurementZ.getUnit(),
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
            final Speed measurementX, final Speed measurementY, final Speed measurementZ) {
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
    public Speed getMeasurementNorm() {
        return new Speed(getNorm(), getUnit());
    }

    /**
     * Creates and returns a new instance having exactly the same contents
     * as this instance.
     *
     * @return a copy of this instance.
     */
    @Override
    public SpeedTriad copy() {
        final var result = new SpeedTriad();
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
        final var result = (SpeedTriad) super.clone();
        copyTo(result);
        return result;
    }
}
