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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

/**
 * Fixes acceleration values taking into account provided bias and cross coupling errors.
 */
@SuppressWarnings("DuplicatedCode")
public class AccelerationFixer {
    /**
     * Identity matrix to be reused.
     */
    private Matrix identity;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix tmp1;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix tmp2;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix diff;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix tmp3;

    /**
     * Measured specific force array to be reused.
     */
    private final double[] measuredF = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing result values to be reused.
     */
    private final double[] res = new double[BodyKinematics.COMPONENTS];

    /**
     * Bias matrix to be reused.
     */
    private Matrix bias;

    /**
     * Cross coupling errors matrix to be reused.
     */
    private Matrix crossCouplingErrors;

    /**
     * Constructor.
     */
    public AccelerationFixer() {
        try {
            identity = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            tmp1 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            tmp2 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            diff = new Matrix(BodyKinematics.COMPONENTS, 1);
            tmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);

            bias = new Matrix(BodyKinematics.COMPONENTS, 1);
            crossCouplingErrors = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public Matrix getBias() {
        return new Matrix(bias);
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result will be stored.
     */
    public void getBias(final Matrix result) {
        bias.copyTo(result);
    }

    /**
     * Sets bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second.
     *             Must be 3x1.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void setBias(final Matrix bias) {
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        this.bias = bias;
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public double[] getBiasArray() {
        final var result = new double[BodyKinematics.COMPONENTS];
        getBiasArray(result);
        return result;
    }

    /**
     * Gets bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void getBiasArray(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            bias.toArray(result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Sets bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second (m/s^2).
     *             Must have length 3.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    public void setBias(final double[] bias) {
        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        try {
            this.bias.fromArray(bias);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Gets acceleration bias.
     *
     * @return acceleration bias.
     */
    public AccelerationTriad getBiasAsTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                bias.getElementAtIndex(0), bias.getElementAtIndex(1), bias.getElementAtIndex(2));
    }

    /**
     * Gets acceleration bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AccelerationTriad result) {
        result.setValueCoordinates(bias);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets acceleration bias.
     *
     * @param bias acceleration bias to be set.
     */
    public void setBias(final AccelerationTriad bias) {
        final var biasX = convertAcceleration(bias.getValueX(), bias.getUnit());
        final var biasY = convertAcceleration(bias.getValueY(), bias.getUnit());
        final var biasZ = convertAcceleration(bias.getValueZ(), bias.getUnit());
        this.bias.setElementAtIndex(0, biasX);
        this.bias.setElementAtIndex(1, biasY);
        this.bias.setElementAtIndex(2, biasZ);
    }

    /**
     * Gets x-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasX() {
        return bias.getElementAtIndex(0);
    }

    /**
     * Sets x-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasX x-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasX(final double biasX) {
        bias.setElementAtIndex(0, biasX);
    }

    /**
     * Gets y-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasY() {
        return bias.getElementAtIndex(1);
    }

    /**
     * Sets y-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasY y-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasY(final double biasY) {
        bias.setElementAtIndex(1, biasY);
    }

    /**
     * Gets z-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @return z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     */
    public double getBiasZ() {
        return bias.getElementAtIndex(2);
    }

    /**
     * Sets z-coordinate of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasZ z-coordinate of bias expressed in meters per squared
     *              second (m/s^2).
     */
    public void setBiasZ(final double biasZ) {
        bias.setElementAtIndex(2, biasZ);
    }

    /**
     * Sets coordinates of bias expressed in meters per squared second
     * (m/s^2).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setBias(final double biasX, final double biasY, final double biasZ) {
        setBiasX(biasX);
        setBiasY(biasY);
        setBiasZ(biasZ);
    }

    /**
     * Gets x-coordinate of bias.
     *
     * @return x-coordinate of bias.
     */
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(getBiasX(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(getBiasX());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     */
    public void setBiasX(final Acceleration biasX) {
        setBiasX(convertAcceleration(biasX));
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(getBiasY(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(getBiasY());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     */
    public void setBiasY(final Acceleration biasY) {
        setBiasY(convertAcceleration(biasY));
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(getBiasZ(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(getBiasZ());
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     */
    public void setBiasZ(final Acceleration biasZ) {
        setBiasZ(convertAcceleration(biasZ));
    }

    /**
     * Sets coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     */
    public void setBias(final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        setBiasX(biasX);
        setBiasY(biasY);
        setBiasZ(biasZ);
    }

    /**
     * Gets cross coupling errors matrix.
     *
     * @return cross coupling errors matrix.
     */
    public Matrix getCrossCouplingErrors() {
        return new Matrix(crossCouplingErrors);
    }

    /**
     * Gets cross coupling errors matrix.
     *
     * @param result instance where result will be stored.
     */
    public void getCrossCouplingErrors(final Matrix result) {
        crossCouplingErrors.copyTo(result);
    }

    /**
     * Sets cross coupling errors matrix.
     *
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided matrix cannot be inverted.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setCrossCouplingErrors(final Matrix crossCouplingErrors) throws AlgebraException {
        if (crossCouplingErrors.getRows() != BodyKinematics.COMPONENTS
                || crossCouplingErrors.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        this.crossCouplingErrors = crossCouplingErrors;

        identity.add(crossCouplingErrors, tmp1);

        Utils.inverse(tmp1, tmp2);
    }

    /**
     * Gets x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getSx() {
        return crossCouplingErrors.getElementAt(0, 0);
    }

    /**
     * Sets x scaling factor
     *
     * @param sx x scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSx(final double sx) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 0, sx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getSy() {
        return crossCouplingErrors.getElementAt(1, 1);
    }

    /**
     * Sets y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSy(final double sy) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(1, 1, sy);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getSz() {
        return crossCouplingErrors.getElementAt(2, 2);
    }

    /**
     * Sets z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setSz(final double sz) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(2, 2, sz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets x-y cross coupling error.
     *
     * @return x-y cross coupling error.
     */
    public double getMxy() {
        return crossCouplingErrors.getElementAt(0, 1);
    }

    /**
     * Sets x-y cross coupling error.
     *
     * @param mxy x-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMxy(final double mxy) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 1, mxy);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets x-z cross coupling error.
     *
     * @return x-z cross coupling error.
     */
    public double getMxz() {
        return crossCouplingErrors.getElementAt(0, 2);
    }

    /**
     * Sets x-z cross coupling error.
     *
     * @param mxz x-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMxz(final double mxz) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 2, mxz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y-x cross coupling error.
     *
     * @return y-x cross coupling error.
     */
    public double getMyx() {
        return crossCouplingErrors.getElementAt(1, 0);
    }

    /**
     * Sets y-x cross coupling error.
     *
     * @param myx y-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMyx(final double myx) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(1, 0, myx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets y-z cross coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getMyz() {
        return crossCouplingErrors.getElementAt(1, 2);
    }

    /**
     * Sets y-z cross coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMyz(final double myz) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(1, 2, myz);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z-x cross coupling error.
     *
     * @return z-x cross coupling error.
     */
    public double getMzx() {
        return crossCouplingErrors.getElementAt(2, 0);
    }

    /**
     * Sets z-x cross coupling error.
     *
     * @param mzx z-x cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMzx(final double mzx) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(2, 0, mzx);
        setCrossCouplingErrors(m);
    }

    /**
     * Gets z-y cross coupling error.
     *
     * @return z-y cross coupling error.
     */
    public double getMzy() {
        return crossCouplingErrors.getElementAt(2, 1);
    }

    /**
     * Sets z-y cross coupling error.
     *
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided value makes cross coupling matrix
     *                          non invertible.
     */
    public void setMzy(final double mzy) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setScalingFactors(final double sx, final double sy, final double sz) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 0, sx);
        m.setElementAt(1, 1, sy);
        m.setElementAt(2, 2, sz);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets cross coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setCrossCouplingErrors(
            final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 1, mxy);
        m.setElementAt(0, 2, mxz);
        m.setElementAt(1, 0, myx);
        m.setElementAt(1, 2, myz);
        m.setElementAt(2, 0, mzx);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Sets scaling factors and cross coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws AlgebraException if provided values make cross coupling matrix
     *                          non invertible.
     */
    public void setScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy) throws AlgebraException {
        final var m = new Matrix(Triad.COMPONENTS, Triad.COMPONENTS);
        m.copyFrom(crossCouplingErrors);
        m.setElementAt(0, 0, sx);
        m.setElementAt(1, 1, sy);
        m.setElementAt(2, 2, sz);
        m.setElementAt(0, 1, mxy);
        m.setElementAt(0, 2, mxz);
        m.setElementAt(1, 0, myx);
        m.setElementAt(1, 2, myz);
        m.setElementAt(2, 0, mzx);
        m.setElementAt(2, 1, mzy);
        setCrossCouplingErrors(m);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force.
     * @param result    instance where restored true specific force will be stored.
     *                  Mut have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if length of provided result array is
     *                                  not 3.
     */
    public void fix(final AccelerationTriad measuredF, final double[] result) throws AlgebraException {
        if (result.length != Triad.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        final var measuredFx = convertAcceleration(measuredF.getValueX(), measuredF.getUnit());
        final var measuredFy = convertAcceleration(measuredF.getValueY(), measuredF.getUnit());
        final var measuredFz = convertAcceleration(measuredF.getValueZ(), measuredF.getUnit());
        fix(measuredFx, measuredFy, measuredFz, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force.
     * @param result    instance where restored true specific force will be stored.
     *                  Mut be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result matrix is not 3x1.
     */
    public void fix(final AccelerationTriad measuredF, final Matrix result) throws AlgebraException {
        if (result.getRows() != Triad.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force.
     * @param result    instance where restored true specific force will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final AccelerationTriad measuredF, final AccelerationTriad result) throws AlgebraException {
        fix(measuredF, this.res);
        result.setValueCoordinates(this.res);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force.
     * @param measuredFy y-coordinate of measured specific force.
     * @param measuredFz z-coordinate of measured specific force.
     * @param result     instance where restored true specific force
     *                   will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public void fix(final Acceleration measuredFx, final Acceleration measuredFy, final Acceleration measuredFz,
                    final AccelerationTriad result) throws AlgebraException {
        final var fx = convertAcceleration(measuredFx);
        final var fy = convertAcceleration(measuredFy);
        final var fz = convertAcceleration(measuredFz);
        fix(fx, fy, fz, this.res);
        result.setValueCoordinates(this.res);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must have length 3.
     * @param result    instance where restored true specific force will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final double[] measuredF, final double[] result) throws AlgebraException {
        if (measuredF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The accelerometer model is
        // fmeas = ba + (I + Ma) * ftrue

        // where:
        // fmeas is measured specific force (vector 3x1)
        // ba is accelerometer bias (vector 3x1)
        // I is the 3x3 identity
        // Ma is the 3x3 cross couplings matrix

        // Hence:
        // ftrue = (I + Ma)^-1 * (fmeas - ba)
        for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
            diff.setElementAtIndex(i, measuredF[i] - bias.getElementAtIndex(i));
        }

        tmp2.multiply(diff, tmp3);

        tmp3.toArray(result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @param result    instance where restored true specific force will be
     *                  stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final double[] result) throws AlgebraException {

        if (measuredF.getRows() != BodyKinematics.COMPONENTS || measuredF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF.getBuffer(), result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @param result    instance where restored true specific force will be
     *                  stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param result     instance where restored true specific force
     *                   will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result array does not have
     *                                  length 3.
     */
    public void fix(final double measuredFx, final double measuredFy, final double measuredFz, final double[] result)
            throws AlgebraException {

        measuredF[0] = measuredFx;
        measuredF[1] = measuredFy;
        measuredF[2] = measuredFz;

        fix(measuredF, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param result     instance where restored true specific force
     *                   will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided result matrix is not 3x1.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredFx, measuredFy, measuredFz, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double[] measuredF, final Matrix bias, final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {
        if (measuredF.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        if (crossCouplingErrors.getRows() != BodyKinematics.COMPONENTS
                || crossCouplingErrors.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        // The accelerometer model is
        // fmeas = ba + (I + Ma) * ftrue

        // where:
        // fmeas is measured specific force (vector 3x1)
        // ba is accelerometer bias (vector 3x1)
        // I is the 3x3 identity
        // Ma is the 3x3 cross couplings matrix

        // Hence:
        // ftrue = (I + Ma)^-1 * (fmeas - ba)
        identity.add(crossCouplingErrors, tmp1);

        Utils.inverse(tmp1, tmp2);

        for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
            diff.setElementAtIndex(i, measuredF[i] - bias.getElementAtIndex(i));
        }

        tmp2.multiply(diff, tmp3);

        tmp3.toArray(result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must be 3x1.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final Matrix bias, final Matrix crossCouplingErrors, final double[] result)
            throws AlgebraException {

        if (measuredF.getRows() != BodyKinematics.COMPONENTS || measuredF.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF.getBuffer(), bias, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(final Matrix measuredF, final Matrix bias, final Matrix crossCouplingErrors, final Matrix result)
            throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredF, bias, crossCouplingErrors, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ, final Matrix crossCouplingErrors,
            final double[] result) throws AlgebraException {

        measuredF[0] = measuredFx;
        measuredF[1] = measuredFy;
        measuredF[2] = measuredFz;

        bias.setElementAtIndex(0, biasX);
        bias.setElementAtIndex(1, biasY);
        bias.setElementAtIndex(2, biasZ);

        fix(measuredF, bias, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @param result              instance where restored true specific force
     *                            will be stored. Must be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ, final Matrix crossCouplingErrors,
            final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, crossCouplingErrors, result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true specific force
     *                   will be stored. Must have length 3.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result does not have length 3.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz, final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy, final double[] result) throws AlgebraException {

        crossCouplingErrors.setElementAt(0, 0, sx);
        crossCouplingErrors.setElementAt(1, 1, sy);
        crossCouplingErrors.setElementAt(2, 2, sz);
        crossCouplingErrors.setElementAt(0, 1, mxy);
        crossCouplingErrors.setElementAt(0, 2, mxz);
        crossCouplingErrors.setElementAt(1, 0, myx);
        crossCouplingErrors.setElementAt(1, 2, myz);
        crossCouplingErrors.setElementAt(2, 0, mzx);
        crossCouplingErrors.setElementAt(2, 1, mzy);

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, crossCouplingErrors, result);
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @param result     instance where restored true specific force
     *                   will be stored. Must have be 3x1.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if result is not 3x1.
     */
    public void fix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy, final Matrix result) throws AlgebraException {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                result.getBuffer());
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force.
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public AccelerationTriad fixAndReturnNew(final AccelerationTriad measuredF) throws AlgebraException {
        final var result = new AccelerationTriad();
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force.
     * @param measuredFy y-coordinate of measured specific force.
     * @param measuredFz z-coordinate of measured specific force.
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public AccelerationTriad fixAndReturnNew(
            final Acceleration measuredFx, final Acceleration measuredFy, final Acceleration measuredFz)
            throws AlgebraException {
        final var result = new AccelerationTriad();
        fix(measuredFx, measuredFy, measuredFz, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must have length 3.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public double[] fixAndReturnNew(final double[] measuredF) throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(final Matrix measuredF) throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredF measured specific force expressed in meters
     *                  per squared second (m/s^2). Must be 3x1.
     * @return restored true specific force.
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(final Matrix measuredF) throws AlgebraException {

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fix(measuredF, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz) throws AlgebraException {
        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     * This method uses last provided bias and cross coupling errors.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @return restored true specific force.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz) throws AlgebraException {
        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double[] measuredF, final Matrix bias, final Matrix crossCouplingErrors) throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must be 3x1.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final Matrix measuredF, final Matrix bias, final Matrix crossCouplingErrors) throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredF           measured specific force expressed in meters
     *                            per squared second (m/s^2). Must have
     *                            length 3.
     * @param bias                bias values expressed in meters per squared
     *                            second (m/s^2). Must be 3x1.
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final Matrix measuredF, final Matrix bias, final Matrix crossCouplingErrors) throws AlgebraException {

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fix(measuredF, bias, crossCouplingErrors, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ, final Matrix crossCouplingErrors)
            throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx          x-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFy          y-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param measuredFz          z-coordinate of measured specific force
     *                            expressed in meters per squared second
     *                            (m/s^2).
     * @param biasX               x-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasY               y-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param biasZ               z-coordinate of bias expressed in
     *                            meters per squared second (m/s^2).
     * @param crossCouplingErrors cross coupling errors matrix. Must be 3x3.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException         if there are numerical instabilities.
     * @throws IllegalArgumentException if any of the provided parameters does
     *                                  not have proper size.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ, final Matrix crossCouplingErrors)
            throws AlgebraException {

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, crossCouplingErrors, result);
        return result;
    }


    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public double[] fixAndReturnNew(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy) throws AlgebraException {

        final var result = new double[BodyKinematics.COMPONENTS];
        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);
        return result;
    }

    /**
     * Fixes provided measured specific force values by undoing the errors
     * introduced by the accelerometer model to restore the true specific
     * force.
     *
     * @param measuredFx x-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFy y-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param measuredFz z-coordinate of measured specific force
     *                   expressed in meters per squared second
     *                   (m/s^2).
     * @param biasX      x-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasY      y-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of bias expressed in
     *                   meters per squared second (m/s^2).
     * @param sx         x scaling factor.
     * @param sy         y scaling factor.
     * @param sz         z scaling factor.
     * @param mxy        x-y cross coupling error.
     * @param mxz        x-z cross coupling error.
     * @param myx        y-x cross coupling error.
     * @param myz        y-z cross coupling error.
     * @param mzx        z-x cross coupling error.
     * @param mzy        z-y cross coupling error.
     * @return restored true specific force expressed in meters per squared
     * second (m/s^2).
     * @throws AlgebraException if there are numerical instabilities.
     */
    public Matrix fixAndReturnNewMatrix(
            final double measuredFx, final double measuredFy, final double measuredFz,
            final double biasX, final double biasY, final double biasZ,
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz, final double myx,
            final double myz, final double mzx, final double mzy) throws AlgebraException {

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);
        return result;
    }

    /**
     * Converts acceleration value and unit to meters per squared second (m/s^2).
     *
     * @param value value to be converted.
     * @param unit  unit of value to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final double value, final AccelerationUnit unit) {
        return AccelerationConverter.convert(value, unit, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts acceleration measurement to meters per squared second (m/s^2).
     *
     * @param acceleration acceleration to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(), acceleration.getUnit());
    }
}
