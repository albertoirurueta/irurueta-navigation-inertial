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

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class WorldMagneticModelTest {

    @Test
    public void testConstructor() {
        final WorldMagneticModel model = new WorldMagneticModel();

        assertEquals(12, WorldMagneticModel.MAX_ORDER);
        assertEquals(5.0, WorldMagneticModel.LIFESPAN, 0.0);
        assertEquals(13, WorldMagneticModel.N);

        assertNotNull(model.c);
        assertNotNull(model.cd);
        assertEquals(0.0, model.epoch, 0.0);
        assertNotNull(model.snorm);
        assertNotNull(model.k);
        assertNotNull(model.fn);
        assertNotNull(model.fm);
    }
}
