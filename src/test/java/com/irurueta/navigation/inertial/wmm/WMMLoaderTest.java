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
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WMMLoaderTest {

    private static final String RESOURCE = "wmm.cof";

    private static final String URL = "https://mobbio.s3.amazonaws.com/wmm.cof";

    private static final String FILE_PATH = "./src/main/resources/com/irurueta/navigation/inertial/wmm/wmm.cof";

    private static final long CHILD_JVM_TIMEOUT_SECONDS = 30;

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
     * Regression check: loading the bundled model from an in-memory stream of its raw bytes
     * yields the same coefficients as loading it from the file.
     * This does not exercise the charset on its own, because both paths decode through the same
     * reader; see {@link #load_whenPlatformDefaultCharsetIsUtf16_decodesModelAsUtf8()} for that.
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
    }

    /**
     * Checks that the model is decoded as UTF-8 even when the platform default charset is not
     * ASCII-compatible.
     * The default charset is fixed at JVM start-up, so this test loads the model in a child JVM
     * started with {@code -Dfile.encoding=UTF-16}. Reading with the platform default charset
     * would decode the ASCII coefficient file as UTF-16 garbage, so the child JVM would either
     * report a different epoch or never finish parsing (it is killed after a timeout).
     *
     * @throws IOException          if the model file cannot be read or the child JVM cannot be started.
     * @throws InterruptedException if interrupted while waiting for the child JVM.
     */
    @Test
    void load_whenPlatformDefaultCharsetIsUtf16_decodesModelAsUtf8() throws IOException, InterruptedException {
        final var expected = WMMLoader.load(new File(FILE_PATH));

        final var java = Path.of(System.getProperty("java.home"), "bin", "java").toString();
        final var outputFile = Files.createTempFile("wmm-loader-charset", ".txt");
        try {
            // child output goes to a file, so a child that hangs (e.g. while parsing a wrongly decoded
            // model) cannot block this test on a pipe; it is killed after the timeout instead.
            final var process = new ProcessBuilder(java,
                    "-Dfile.encoding=UTF-16",
                    "-Dstdout.encoding=UTF-8",
                    "-cp", System.getProperty("java.class.path"),
                    EpochPrinter.class.getName(),
                    FILE_PATH)
                    .redirectErrorStream(true)
                    .redirectOutput(outputFile.toFile())
                    .start();
            final var finished = process.waitFor(CHILD_JVM_TIMEOUT_SECONDS, TimeUnit.SECONDS);
            if (!finished) {
                process.destroyForcibly().waitFor();
            }
            final var output = Files.readString(outputFile, StandardCharsets.UTF_8);
            assertTrue(finished, "child JVM did not finish loading the model: " + output);

            final var lines = output.lines().toList();
            assertEquals(0, process.exitValue(), output);
            assertEquals("UTF-16", lines.get(0), "child JVM default charset");
            assertEquals(expected.getEpoch(), Double.parseDouble(lines.get(1)), 0.0);
        } finally {
            Files.deleteIfExists(outputFile);
        }
    }

    /**
     * Entry point run in a child JVM by
     * {@link #load_whenPlatformDefaultCharsetIsUtf16_decodesModelAsUtf8()}: prints the default
     * charset and the epoch of the model loaded from the file given as first argument.
     */
    static final class EpochPrinter {

        private EpochPrinter() {
        }

        /**
         * Prints the default charset and the epoch of the loaded model.
         *
         * @param args first argument is the path of the model file.
         * @throws IOException if the model file cannot be read.
         */
        public static void main(final String[] args) throws IOException {
            System.out.println(Charset.defaultCharset().name());
            try (var stream = new FileInputStream(args[0])) {
                System.out.println(WMMLoader.load(stream).getEpoch());
            }
        }
    }
}
