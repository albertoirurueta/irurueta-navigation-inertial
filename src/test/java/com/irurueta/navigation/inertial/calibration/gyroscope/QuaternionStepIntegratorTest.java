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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class QuaternionStepIntegratorTest {

    @Test
    void create_whenEulerType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.EULER_METHOD);

        // check
        assertEquals(QuaternionStepIntegratorType.EULER_METHOD, integrator.getType());
        assertInstanceOf(EulerQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenMidPointType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.MID_POINT);

        // check
        assertEquals(QuaternionStepIntegratorType.MID_POINT, integrator.getType());
        assertInstanceOf(MidPointQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenSuhType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.SUH);

        // check
        assertEquals(QuaternionStepIntegratorType.SUH, integrator.getType());
        assertInstanceOf(SuhQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenTrawnyType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.TRAWNY);

        // check
        assertEquals(QuaternionStepIntegratorType.TRAWNY, integrator.getType());
        assertInstanceOf(TrawnyQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenYuanType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.YUAN);

        // check
        assertEquals(QuaternionStepIntegratorType.YUAN, integrator.getType());
        assertInstanceOf(YuanQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenRungeKuttaType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.RUNGE_KUTTA);

        // check
        assertEquals(QuaternionStepIntegratorType.RUNGE_KUTTA, integrator.getType());
        assertInstanceOf(RungeKuttaQuaternionStepIntegrator.class, integrator);
    }

    @Test
    void create_whenDefaultType_returnsExpectedIntegratorType() {
        final var integrator = QuaternionStepIntegrator.create();

        // check
        assertEquals(QuaternionStepIntegratorType.RUNGE_KUTTA, integrator.getType());
        assertInstanceOf(RungeKuttaQuaternionStepIntegrator.class, integrator);
    }
}
