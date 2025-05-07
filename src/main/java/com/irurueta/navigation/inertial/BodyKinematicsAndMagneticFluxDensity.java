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

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Contains body kinematics describing the forces and angular rate applied to a body,
 * along with the sensed magnetic flux density resolved around body coordinates.
 */
public class BodyKinematicsAndMagneticFluxDensity implements Serializable, Cloneable {

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Body kinematics containing sensed specific force and angular rate.
     */
    private BodyKinematics kinematics;

    /**
     * Body magnetic flux density.
     */
    private BodyMagneticFluxDensity magneticFluxDensity;

    /**
     * Constructor.
     */
    public BodyKinematicsAndMagneticFluxDensity() {
    }

    /**
     * Constructor.
     *
     * @param kinematics body kinematics containing sensed specific force
     *                   and angular rate.
     */
    public BodyKinematicsAndMagneticFluxDensity(final BodyKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Constructor.
     *
     * @param magneticFluxDensity body magnetic flux density.
     */
    public BodyKinematicsAndMagneticFluxDensity(final BodyMagneticFluxDensity magneticFluxDensity) {
        this.magneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param kinematics          body kinematics containing sensed specific force
     *                            and angular rate.
     * @param magneticFluxDensity body magnetic flux density.
     */
    public BodyKinematicsAndMagneticFluxDensity(
            final BodyKinematics kinematics, final BodyMagneticFluxDensity magneticFluxDensity) {
        this.kinematics = kinematics;
        this.magneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public BodyKinematicsAndMagneticFluxDensity(final BodyKinematicsAndMagneticFluxDensity input) {
        copyFrom(input);
    }

    /**
     * Gets body kinematics containing sensed specific force and angular rate.
     *
     * @return body kinematics containing sensed specific force and angular rate.
     */
    public BodyKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Sets body kinematics containing sensed specific force and angular rate.
     *
     * @param kinematics body kinematics containing sensed specific force and
     *                   angular rate.
     */
    public void setKinematics(final BodyKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Gets body magnetic flux density.
     *
     * @return body magnetic flux density.
     */
    public BodyMagneticFluxDensity getMagneticFluxDensity() {
        return magneticFluxDensity;
    }

    /**
     * Sets body magnetic flux density.
     *
     * @param magneticFluxDensity body magnetic flux density.
     */
    public void setMagneticFluxDensity(final BodyMagneticFluxDensity magneticFluxDensity) {
        this.magneticFluxDensity = magneticFluxDensity;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final BodyKinematicsAndMagneticFluxDensity input) {
        if (input.kinematics != null) {
            if (kinematics == null) {
                kinematics = new BodyKinematics(input.kinematics);
            } else {
                kinematics.copyFrom(input.kinematics);
            }
        } else {
            kinematics = null;
        }

        if (input.magneticFluxDensity != null) {
            if (magneticFluxDensity == null) {
                magneticFluxDensity = new BodyMagneticFluxDensity(input.magneticFluxDensity);
            } else {
                magneticFluxDensity.copyFrom(input.magneticFluxDensity);
            }
        } else {
            magneticFluxDensity = null;
        }
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final BodyKinematicsAndMagneticFluxDensity output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fas classification and storage of objects in
     * collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(kinematics, magneticFluxDensity);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final BodyKinematicsAndMagneticFluxDensity other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between contents.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final BodyKinematicsAndMagneticFluxDensity other, final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.kinematics == null && kinematics == null)
                || (kinematics != null && kinematics.equals(other.kinematics, threshold)))
                && ((other.magneticFluxDensity == null && magneticFluxDensity == null)
                || (magneticFluxDensity != null && magneticFluxDensity.equals(other.magneticFluxDensity, threshold)));
    }

    /**
     * Checks if provided object is a BodyKinematicsAndMagneticFluxDensity instance
     * having exactly the same contents as this instance.
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

        final var other = (BodyKinematicsAndMagneticFluxDensity) obj;
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
        final var result = (BodyKinematicsAndMagneticFluxDensity) super.clone();
        copyTo(result);
        return result;
    }
}
