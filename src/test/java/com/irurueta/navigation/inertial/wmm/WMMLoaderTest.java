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

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URL;

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
}
