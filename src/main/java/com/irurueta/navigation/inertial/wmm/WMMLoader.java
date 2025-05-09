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

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.StreamTokenizer;
import java.net.HttpURLConnection;
import java.net.URL;

/**
 * Loads a WWM from a file of coefficients.
 * The file of coefficients is updated every 5 years and can be obtained
 * at: <a href="https://www.ngdc.noaa.gov/geomag/WMM/">https://www.ngdc.noaa.gov/geomag/WMM/</a>
 *
 * @see WorldMagneticModel
 */
public class WMMLoader {

    /**
     * Maximum allowed value within file of coefficients.
     */
    private static final double MAX_VAL = 9999.0;

    /**
     * Number of coefficients.
     */
    private static final int N = WorldMagneticModel.N;

    /**
     * Constructor.
     * Prevents instantiation of helper class.
     */
    private WMMLoader() {
    }

    /**
     * Loads World Magnetic Model from provided resource name.
     * Resource will be resolved an loaded using current class loader.
     *
     * @param resource a resource name.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel loadFromResource(final String resource) throws IOException {
        try (final var stream = WMMLoader.class.getResourceAsStream(resource)) {
            return load(stream);
        }
    }

    /**
     * Loads World Magnetic Model from provided URL.
     * Data will be requested with a "GET" method without additional headers.
     *
     * @param url URL to request data from.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel loadFromUrl(final String url) throws IOException {
        return load(new URL(url));
    }

    /**
     * Loads World Magnetic Model from provided file path.
     *
     * @param filePath a file path.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel loadFromFile(final String filePath) throws IOException {
        try (final var stream = new FileInputStream(filePath)) {
            return load(stream);
        }
    }

    /**
     * Loads World Magnetic Model from provided URL.
     * Data will be requested with a "GET" method without additional headers.
     *
     * @param url URL to request data from.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel load(final URL url) throws IOException {
        final var connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        try (final var stream = connection.getInputStream()) {
            return load(stream);
        }
    }

    /**
     * Loads World Magnetic Model from provided file.
     *
     * @param file a file.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel load(final File file) throws IOException {
        try (final var stream = new FileInputStream(file)) {
            return load(stream);
        }
    }

    /**
     * Loads World Magnetic Model from provided stream of data.
     *
     * @param stream a stream of data.
     * @return a World Magnetic Model containing all required coefficients.
     * @throws IOException if an I/O error occurs.
     */
    public static WorldMagneticModel load(final InputStream stream) throws IOException {
        try (final var reader = new InputStreamReader(stream)) {
            final var result = new WorldMagneticModel();
            final var tokenizer = new StreamTokenizer(reader);

            // Read World Magnetic Model spherical harmonic coefficients
            result.snorm[0] = 1.0;
            result.c[0][0] = 0.0;
            result.cd[0][0] = 0.0;

            tokenizer.nextToken();
            result.epoch = tokenizer.nval;
            tokenizer.nextToken();
            tokenizer.nextToken();

            // loop to get data from file
            while (true) {
                tokenizer.nextToken();
                if (tokenizer.nval >= MAX_VAL) {
                    // end of file
                    break;
                }

                final var n = (int) tokenizer.nval;
                tokenizer.nextToken();
                final var m = (int) tokenizer.nval;
                tokenizer.nextToken();
                final var gnm = tokenizer.nval;
                tokenizer.nextToken();
                final var hnm = tokenizer.nval;
                tokenizer.nextToken();
                final var dgnm = tokenizer.nval;
                tokenizer.nextToken();
                final var dhnm = tokenizer.nval;

                if (m <= n) {
                    result.c[m][n] = gnm;
                    result.cd[m][n] = dgnm;

                    if (m != 0) {
                        result.c[n][m - 1] = hnm;
                        result.cd[n][m - 1] = dhnm;
                    }
                }
            }

            // convert Schmidt normalized Gauss coefficients to un-normalized
            result.snorm[0] = 1.0;
            for (var n = 1; n <= WorldMagneticModel.MAX_ORDER; n++) {

                result.snorm[n] = result.snorm[n - 1] * (2 * n - 1) / n;
                var j = 2;

                for (int m = 0, D1 = 1, D2 = (n - m + D1) / D1; D2 > 0; D2--, m += D1) {
                    result.k[m][n] = (double) (((n - 1) * (n - 1)) - (m * m)) / (double) ((2 * n - 1) * (2 * n - 3));
                    if (m > 0) {
                        final var flnmj = ((n - m + 1) * j) / (double) (n + m);
                        result.snorm[n + m * N] = result.snorm[n + (m - 1) * N] * Math.sqrt(flnmj);
                        j = 1;
                        result.c[n][m - 1] = result.snorm[n + m * N] * result.c[n][m - 1];
                        result.cd[n][m - 1] = result.snorm[n + m * N] * result.cd[n][m - 1];
                    }
                    result.c[m][n] = result.snorm[n + m * N] * result.c[m][n];
                    result.cd[m][n] = result.snorm[n + m * N] * result.cd[m][n];
                }

                result.fn[n] = (n + 1);
                result.fm[n] = n;
            }

            result.k[1][1] = 0.0;

            return result;
        }
    }
}
