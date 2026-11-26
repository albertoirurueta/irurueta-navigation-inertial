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
package com.irurueta.navigation.inertial.wmm;

/**
 * Contains data defining the World Magnetic Model (WWM).
 * <p>
 * The WWM defines the Earth's magnetic field in geodetic coordinates
 * (latitude, longitude and height) respect the WGS84 spheroid from
 * the coefficients of the current official Department of Defense (DOD)
 * spherical harmonic world magnetic model (WMM-2010).
 * The WWM series of models is updated every 5 years on January 1st of
 * those years, which are divisible by 5 (i.e. 1980, 1985, 1990, etc.)
 * by the Naval Oceanographic Office in cooperation with the
 * British Geological Survey (BGS).
 * <p>
 * The model is based on geomagnetic survey measurements from aircraft,
 * satellite and geomagnetic observatories.
 */
public class WorldMagneticModel {

    /**
     * The maximum number of degrees of the spherical harmonic model.
     */
    public static final int MAX_ORDER = 12;

    /**
     * The lifespan of a WMM is 5 years.
     */
    public static final double LIFESPAN = 5.0;

    /**
     * Number of coefficients.
     */
    static final int N = 13;

    /**
     * Squared number of coefficients.
     */
    private static final int N2 = N * N;

    /**
     * The Gauss coefficients of main geomagnetic model (nt).
     */
    private final double[][] c = new double[N][N];

    /**
     * The Gauss coefficients of secular geomagnetic model (nt/yr).
     */
    private final double[][] cd = new double[N][N];

    /**
     * The date in years, for the start of the valid time of the fit coefficients.
     */
    private double epoch;

    /**
     * The Schmidt normalization factors.
     */
    private final double[] snorm = new double[N2];

    /**
     * The sine of (m*spherical coord. longitude).
     */
    private final double[][] k = new double[N][N];

    /**
     * The cosine of (m*spherical coord. longitude).
     */
    private final double[] fn = new double[N];

    /**
     * The sine of (m*spherical coord. latitude).
     */
    private final double[] fm = new double[N];

    /**
     * Gets the Gauss coefficients of main geomagnetic model (nt).
     *
     * @return the Gauss coefficients of main geomagnetic model (nt).
     */
    double[][] getC() {
        return c;
    }

    /**
     * Gets the Gauss coefficients of secular geomagnetic model (nt/yr).
     *
     * @return the Gauss coefficients of secular geomagnetic model (nt/yr).
     */
    double[][] getCd() {
        return cd;
    }

    /**
     * Gets the date in years, for the start of the valid time of the fit coefficients.
     *
     * @return the date in years, for the start of the valid time of the fit coefficients.
     */
    double getEpoch() {
        return epoch;
    }

    /**
     * Updates the date in years, for the start of the valid time of the fit coefficients.
     *
     * @param newEpoch the date in years, for the start of the valid time of the fit coefficients.
     */
    void updateEpoch(final double newEpoch) {
        epoch = newEpoch;
    }

    /**
     * Gets the Schmidt normalization factors.
     *
     * @return the Schmidt normalization factors.
     */
    double[] getSnorm() {
        return snorm;
    }

    /**
     * Gets the sine of (m*spherical coord. longitude).
     *
     * @return the sine of (m*spherical coord. longitude).
     */
    double[][] getK() {
        return k;
    }

    /**
     * Gets the cosine of (m*spherical coord. longitude).
     *
     * @return the cosine of (m*spherical coord. longitude).
     */
    double[] getFn() {
        return fn;
    }

    /**
     * Gets the sine of (m*spherical coord. latitude).
     *
     * @return the sine of (m*spherical coord. latitude).
     */
    double[] getFm() {
        return fm;
    }
}
