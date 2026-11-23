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
import com.irurueta.numerical.EvaluationException;

/**
 * Estimates numerically the Jacobian of a vectorial model function respect to its parameters.
 * Contrary to the generic Jacobian estimator, the step used to perturb each parameter is never
 * smaller than a minimum absolute value, so that the Jacobian does not vanish (loses precision)
 * when a parameter is close to zero, which happens when the true value of a parameter is zero.
 */
class ModelJacobianEstimator {

    /**
     * Relative step used to perturb each parameter.
     */
    private static final double RELATIVE_STEP = 1e-7;

    /**
     * Model function.
     */
    private final ModelFunction function;

    /**
     * Array containing perturbed parameters.
     */
    private double[] perturbed;

    /**
     * Array containing function value at nominal parameters.
     */
    private double[] nominal;

    /**
     * Array containing function value at perturbed parameters.
     */
    private double[] shifted;

    /**
     * Constructor.
     *
     * @param function model function.
     */
    ModelJacobianEstimator(final ModelFunction function) {
        this.function = function;
    }

    /**
     * Estimates the Jacobian of the model at provided parameters.
     *
     * @param params   parameters where the Jacobian is estimated.
     * @param jacobian matrix where the Jacobian is stored. It must have as many rows as the
     *                 function has components and as many columns as parameters.
     * @throws EvaluationException if the model cannot be evaluated.
     */
    void jacobian(final double[] params, final Matrix jacobian) throws EvaluationException {
        final var numParams = params.length;
        final var numVariables = jacobian.getRows();
        if (perturbed == null || perturbed.length != numParams) {
            perturbed = new double[numParams];
        }
        if (nominal == null || nominal.length != numVariables) {
            nominal = new double[numVariables];
            shifted = new double[numVariables];
        }

        System.arraycopy(params, 0, perturbed, 0, numParams);
        function.evaluate(params, nominal);
        for (var j = 0; j < numParams; j++) {
            final var value = params[j];
            perturbed[j] = value + RELATIVE_STEP * Math.max(1.0, Math.abs(value));
            // actual step after rounding
            final var step = perturbed[j] - value;
            function.evaluate(perturbed, shifted);
            perturbed[j] = value;

            for (var i = 0; i < numVariables; i++) {
                jacobian.setElementAt(i, j, (shifted[i] - nominal[i]) / step);
            }
        }
    }

    /**
     * Vectorial model function of the parameters.
     */
    @FunctionalInterface
    interface ModelFunction {
        /**
         * Evaluates the model.
         *
         * @param params parameters of the model.
         * @param result array where the evaluation is stored.
         * @throws EvaluationException if the model cannot be evaluated.
         */
        void evaluate(double[] params, double[] result) throws EvaluationException;
    }
}
