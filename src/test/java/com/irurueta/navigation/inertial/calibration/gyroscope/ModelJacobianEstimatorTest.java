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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.numerical.EvaluationException;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class ModelJacobianEstimatorTest {

    private static final double ABSOLUTE_ERROR = 1e-5;

    @Test
    void testJacobian() throws WrongSizeException, EvaluationException {
        // f1 = p0 * p1, f2 = p0 + 3 * p2 * p2, f3 = sin(p1)
        final var estimator = new ModelJacobianEstimator((params, result) -> {
            result[0] = params[0] * params[1];
            result[1] = params[0] + 3.0 * params[2] * params[2];
            result[2] = Math.sin(params[1]);
        });

        final var params = new double[]{2.0, 0.5, -1.5};
        final var jacobian = new Matrix(3, 3);
        estimator.jacobian(params, jacobian);

        assertEquals(0.5, jacobian.getElementAt(0, 0), ABSOLUTE_ERROR);
        assertEquals(2.0, jacobian.getElementAt(0, 1), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian.getElementAt(0, 2), ABSOLUTE_ERROR);
        assertEquals(1.0, jacobian.getElementAt(1, 0), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian.getElementAt(1, 1), ABSOLUTE_ERROR);
        assertEquals(-9.0, jacobian.getElementAt(1, 2), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian.getElementAt(2, 0), ABSOLUTE_ERROR);
        assertEquals(Math.cos(0.5), jacobian.getElementAt(2, 1), ABSOLUTE_ERROR);
        assertEquals(0.0, jacobian.getElementAt(2, 2), ABSOLUTE_ERROR);

        // estimator can be reused with other parameters
        params[1] = 0.0;
        estimator.jacobian(params, jacobian);
        assertEquals(1.0, jacobian.getElementAt(2, 1), ABSOLUTE_ERROR);
    }

    @Test
    void testJacobianWhenParametersAreCloseToZero() throws WrongSizeException, EvaluationException {
        // a step proportional to the parameter would vanish when the parameter is (almost) zero
        final var estimator = new ModelJacobianEstimator((params, result) -> result[0] = 2.0 * params[0]);

        final var jacobian = new Matrix(1, 1);
        estimator.jacobian(new double[]{1e-17}, jacobian);

        assertEquals(2.0, jacobian.getElementAt(0, 0), ABSOLUTE_ERROR);
    }
}
