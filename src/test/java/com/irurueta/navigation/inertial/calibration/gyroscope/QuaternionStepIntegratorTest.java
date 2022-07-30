/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class QuaternionStepIntegratorTest {

    @Test
    public void create_whenEulerType_returnsExpectedIntegratorType() {
        final QuaternionStepIntegrator integrator = QuaternionStepIntegrator.create(
                QuaternionStepIntegratorType.EULER_METHOD);

        // check
        assertEquals(QuaternionStepIntegratorType.EULER_METHOD, integrator.getType());
        assertTrue(integrator instanceof EulerQuaternionStepIntegrator);
    }

    @Test
    public void create_whenMidPointType_returnsExpectedIntegratorType() {
        final QuaternionStepIntegrator integrator = QuaternionStepIntegrator.create(
                QuaternionStepIntegratorType.MID_POINT);

        // check
        assertEquals(QuaternionStepIntegratorType.MID_POINT, integrator.getType());
        assertTrue(integrator instanceof MidPointQuaternionStepIntegrator);
    }

    @Test
    public void create_whenRungeKuttaType_returnsExpectedIntegratorType() {
        final QuaternionStepIntegrator integrator = QuaternionStepIntegrator.create(
                QuaternionStepIntegratorType.RUNGE_KUTTA);

        // check
        assertEquals(QuaternionStepIntegratorType.RUNGE_KUTTA, integrator.getType());
        assertTrue(integrator instanceof RungeKuttaQuaternionStepIntegrator);
    }

    @Test
    public void create_whenDefaultType_returnsExpectedIntegratorType() {
        final QuaternionStepIntegrator integrator = QuaternionStepIntegrator.create();

        // check
        assertEquals(QuaternionStepIntegratorType.RUNGE_KUTTA, integrator.getType());
        assertTrue(integrator instanceof RungeKuttaQuaternionStepIntegrator);
    }
}
