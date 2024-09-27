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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.Test;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RungeKuttaQuaternionStepIntegratorTest {

    private static final double TIME_INTERVAL = 0.02;

    private static final int NUM_SAMPLES = 50;

    private static final int TIMES = 100;

    private static final double MIN_ANGULAR_SPEED_DEGREES_PER_SECOND = -45.0;

    private static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 45.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;

    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final double SMALL_ABSOLUTE_ERROR = 2e-4;

    @Test
    public void integrate_whenOneStep_computesExpectedResult() throws RotationException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
        final double wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
        final double wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

        final Quaternion initialAttitude = getInitialAttitude();
        final double totalTime = TIME_INTERVAL;
        final Quaternion delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
        final Quaternion expectedResult = initialAttitude.combineAndReturnNew(delta);
        expectedResult.normalize();

        final RungeKuttaQuaternionStepIntegrator integrator = new RungeKuttaQuaternionStepIntegrator();
        final Quaternion result1 = new Quaternion();
        final Quaternion result2 = new Quaternion();
        RungeKuttaQuaternionStepIntegrator.integrationStep(initialAttitude, wx, wy, wz, wx, wy, wz, TIME_INTERVAL,
                result1);
        integrator.integrate(initialAttitude, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);

        assertEquals(result1, result2);
        assertTrue(expectedResult.equals(result1, SMALL_ABSOLUTE_ERROR));
    }

    @Test
    public void integrate_whenMultipleSteps_computesExpectedResult() throws RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer();
            final double wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final double wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final double wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

            final Quaternion initialAttitude = getInitialAttitude();
            final double totalTime = NUM_SAMPLES * TIME_INTERVAL;
            final Quaternion delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
            final Quaternion expectedResult = initialAttitude.combineAndReturnNew(delta);
            expectedResult.normalize();

            final RungeKuttaQuaternionStepIntegrator integrator = new RungeKuttaQuaternionStepIntegrator();
            final Quaternion result1 = new Quaternion(initialAttitude);
            final Quaternion result2 = new Quaternion(initialAttitude);
            for (int i = 0; i < NUM_SAMPLES; i++) {
                RungeKuttaQuaternionStepIntegrator.integrationStep(result1, wx, wy, wz, wx, wy, wz, TIME_INTERVAL,
                        result1);
                integrator.integrate(result2, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);
            }

            assertEquals(result1, result2);

            final double[] resultEulerAngles = result1.toEulerAngles();
            final double[] expectedEulerAngles = expectedResult.toEulerAngles();
            final double[] diffAngles = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultEulerAngles);
            Logger.getLogger(RungeKuttaQuaternionStepIntegratorTest.class.getName())
                    .log(Level.INFO, String.format("Error euler angles: %s",  Arrays.toString(diffAngles)));

            if (!expectedResult.equals(result1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(expectedResult.equals(result1, ABSOLUTE_ERROR));
            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void compareErrors_whenNoNoise_haveExpectedRelativeAccuracy() throws RotationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++){
            final UniformRandomizer randomizer = new UniformRandomizer();
            final double wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final double wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final double wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

            final Quaternion initialAttitude = getInitialAttitude();
            final double totalTime = NUM_SAMPLES * TIME_INTERVAL;
            final Quaternion delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
            final Quaternion expectedResult = initialAttitude.combineAndReturnNew(delta);
            expectedResult.normalize();

            final EulerQuaternionStepIntegrator eulerIntegrator = new EulerQuaternionStepIntegrator();
            final MidPointQuaternionStepIntegrator midPointIntegrator = new MidPointQuaternionStepIntegrator();
            final RungeKuttaQuaternionStepIntegrator rungeKuttaIntegrator = new RungeKuttaQuaternionStepIntegrator();
            final Quaternion result1 = new Quaternion(initialAttitude);
            final Quaternion result2 = new Quaternion(initialAttitude);
            final Quaternion result3 = new Quaternion(initialAttitude);
            for (int i = 0; i < NUM_SAMPLES; i++) {
                eulerIntegrator.integrate(result1, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result1);
                midPointIntegrator.integrate(result2, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);
                rungeKuttaIntegrator.integrate(result3, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result3);
            }

            final double[] resultEulerAngles = result1.toEulerAngles();
            final double[] resultMidPointEulerAngles = result2.toEulerAngles();
            final double[] resultRungeKuttaEulerAngles = result3.toEulerAngles();

            final double[] expectedEulerAngles = expectedResult.toEulerAngles();
            final double[] diffAngles1 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultEulerAngles);
            final double[] diffAngles2 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultMidPointEulerAngles);
            final double[] diffAngles3 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultRungeKuttaEulerAngles);

            final double error1 = Utils.normF(diffAngles1);
            final double error2 = Utils.normF(diffAngles2);
            final double error3 = Utils.normF(diffAngles3);

            Logger.getLogger(RungeKuttaQuaternionStepIntegratorTest.class.getName())
                    .log(Level.INFO, String.format("Euler error: %f radians", error1));
            Logger.getLogger(RungeKuttaQuaternionStepIntegratorTest.class.getName())
                    .log(Level.INFO, String.format("Mid-point error: %f radians", error2));
            Logger.getLogger(RungeKuttaQuaternionStepIntegratorTest.class.getName())
                    .log(Level.INFO, String.format("Runge-Kutta error: %f radians", error3));

            if (error1 < error2 || error1 < error3) {
                continue;
            }

            assertTrue(error1 >= error2);
            assertTrue(error1 >= error3);
            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private static Quaternion getInitialAttitude() {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        return new Quaternion(roll, pitch, yaw);
    }
}
