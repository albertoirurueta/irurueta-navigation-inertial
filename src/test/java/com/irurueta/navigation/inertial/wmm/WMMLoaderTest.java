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

import org.junit.jupiter.api.Test;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Files;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class WMMLoaderTest {

    private static final String RESOURCE = "wmm.cof";

    private static final String URL = "https://mobbio.s3.amazonaws.com/wmm.cof";

    private static final String FILE_PATH = "./src/main/resources/com/irurueta/navigation/inertial/wmm/wmm.cof";

    @Test
    void testLoadFromResource() throws IOException {
        final var model = WMMLoader.loadFromResource(RESOURCE);
        assertNotNull(model);
    }

    @Test
    void testLoadFromUrl() throws IOException {
        final var model = WMMLoader.loadFromUrl(URL);
        assertNotNull(model);
    }

    @Test
    void testLoadFromFile() throws IOException {
        final var model = WMMLoader.loadFromFile(FILE_PATH);
        assertNotNull(model);
    }

    @Test
    void testLoad1() throws IOException {
        final var url = new URL(URL);
        final var model = WMMLoader.load(url);
        assertNotNull(model);
    }

    @Test
    void testLoad2() throws IOException {
        final var file = new File(FILE_PATH);
        final var model = WMMLoader.load(file);
        assertNotNull(model);
    }

    @Test
    void testLoad3() throws IOException {
        final var stream = new FileInputStream(FILE_PATH);
        final var model = WMMLoader.load(stream);
        stream.close();

        assertNotNull(model);
    }

    /**
     * Checks that loading the bundled model from an in-memory stream of its raw bytes yields
     * the same coefficients as loading it from the file, so decoding does not depend on the
     * platform default charset.
     *
     * @throws IOException if the model file cannot be read.
     */
    @Test
    void load_whenStreamOfModelFileBytes_returnsModelEqualToFileModel() throws IOException {
        final var file = new File(FILE_PATH);
        final var bytes = Files.readAllBytes(file.toPath());

        final var expected = WMMLoader.load(file);
        final WorldMagneticModel actual;
        try (var stream = new ByteArrayInputStream(bytes)) {
            actual = WMMLoader.load(stream);
        }

        assertEquals(expected.getEpoch(), actual.getEpoch(), 0.0);
        assertArrayEquals(expected.getC(), actual.getC());
        assertArrayEquals(expected.getCd(), actual.getCd());
        assertArrayEquals(expected.getSnorm(), actual.getSnorm(), 0.0);
        assertEquals(expected.getC()[1][0], actual.getC()[1][0], 0.0);
        assertEquals(expected.getCd()[1][0], actual.getCd()[1][0], 0.0);
    }
}
