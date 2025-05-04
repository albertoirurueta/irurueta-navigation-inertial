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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.statistics.UniformRandomizer;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

class MidPointQuaternionStepIntegratorTest {

    private static final double TIME_INTERVAL = 0.02;

    private static final int NUM_SAMPLES = 50;

    private static final int TIMES = 100;

    private static final double MIN_ANGULAR_SPEED_DEGREES_PER_SECOND = -45.0;

    private static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 45.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;

    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-4;

    @Test
    void integrate_whenOneStep_computesExpectedResult() throws RotationException {
        final var randomizer = new UniformRandomizer();
        final var wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
        final var wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
        final var wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

        final var initialAttitude = getInitialAttitude();
        final var totalTime = TIME_INTERVAL;
        final var delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
        final var expectedResult = initialAttitude.combineAndReturnNew(delta);
        expectedResult.normalize();

        final var integrator = new MidPointQuaternionStepIntegrator();
        final var result1 = new Quaternion();
        final var result2 = new Quaternion();
        MidPointQuaternionStepIntegrator.integrationStep(initialAttitude, wx, wy, wz, wx, wy, wz, TIME_INTERVAL,
                result1);
        integrator.integrate(initialAttitude, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);

        assertEquals(result1, result2);
        assertTrue(expectedResult.equals(result1, SMALL_ABSOLUTE_ERROR));
    }

    @Test
    void integrate_whenMultipleSteps_computesExpectedResult() throws RotationException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND, 
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final var wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND, 
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final var wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND, 
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

            final var initialAttitude = getInitialAttitude();
            final var totalTime = NUM_SAMPLES * TIME_INTERVAL;
            final var delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
            final var expectedResult = initialAttitude.combineAndReturnNew(delta);
            expectedResult.normalize();

            final var integrator = new MidPointQuaternionStepIntegrator();
            final var result1 = new Quaternion(initialAttitude);
            final var result2 = new Quaternion(initialAttitude);
            for (var i = 0; i < NUM_SAMPLES; i++) {
                MidPointQuaternionStepIntegrator.integrationStep(result1, wx, wy, wz, wx, wy, wz, TIME_INTERVAL,
                        result1);
                integrator.integrate(result2, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);
            }

            assertEquals(result1, result2);

            final var resultEulerAngles = result1.toEulerAngles();
            final var expectedEulerAngles = expectedResult.toEulerAngles();
            final var diffAngles = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultEulerAngles);
            Logger.getLogger(MidPointQuaternionStepIntegratorTest.class.getName()).log(Level.INFO, 
                    String.format("Error euler angles: %s", Arrays.toString(diffAngles)));

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
    void compareErrors_whenNoNoise_haveExpectedRelativeAccuracy() throws RotationException {
        var numValid = 0;
        var avgError1 = 0.0;
        var avgError2 = 0.0;
        var avgError3 = 0.0;
        var avgError4 = 0.0;
        var avgError5 = 0.0;
        var avgError6 = 0.0;
        var num = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wx = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND, 
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final var wy = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));
            final var wz = Math.toRadians(randomizer.nextDouble(MIN_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND));

            final var initialAttitude = getInitialAttitude();
            final var totalTime = NUM_SAMPLES * TIME_INTERVAL;
            final var delta = new Quaternion(totalTime * wx, totalTime * wy, totalTime * wz);
            final var expectedResult = initialAttitude.combineAndReturnNew(delta);
            expectedResult.normalize();

            final var suhIntegrator = new SuhQuaternionStepIntegrator();
            final var trawnyIntegrator = new TrawnyQuaternionStepIntegrator();
            final var yuanIntegrator = new YuanQuaternionStepIntegrator();
            final var eulerIntegrator = new EulerQuaternionStepIntegrator();
            final var midPointIntegrator = new MidPointQuaternionStepIntegrator();
            final var rungeKuttaIntegrator = new RungeKuttaQuaternionStepIntegrator();
            final var result1 = new Quaternion(initialAttitude);
            final var result2 = new Quaternion(initialAttitude);
            final var result3 = new Quaternion(initialAttitude);
            final var result4 = new Quaternion(initialAttitude);
            final var result5 = new Quaternion(initialAttitude);
            final var result6 = new Quaternion(initialAttitude);
            for (var i = 0; i < NUM_SAMPLES; i++) {
                suhIntegrator.integrate(result1, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result1);
                trawnyIntegrator.integrate(result2, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result2);
                yuanIntegrator.integrate(result3, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result3);
                eulerIntegrator.integrate(result1, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result4);
                midPointIntegrator.integrate(result2, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result5);
                rungeKuttaIntegrator.integrate(result3, wx, wy, wz, wx, wy, wz, TIME_INTERVAL, result6);
            }

            final var resultSuhAngles = result1.toEulerAngles();
            final var resultTrawnyEulerAngles = result2.toEulerAngles();
            final var resultYuanAngles = result2.toEulerAngles();
            final var resultEulerAngles = result1.toEulerAngles();
            final var resultMidPointEulerAngles = result2.toEulerAngles();
            final var resultRungeKuttaEulerAngles = result3.toEulerAngles();

            final var expectedEulerAngles = expectedResult.toEulerAngles();
            final var diffAngles1 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultSuhAngles);
            final var diffAngles2 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultTrawnyEulerAngles);
            final var diffAngles3 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultYuanAngles);
            final var diffAngles4 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultEulerAngles);
            final var diffAngles5 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultMidPointEulerAngles);
            final var diffAngles6 = ArrayUtils.subtractAndReturnNew(expectedEulerAngles, resultRungeKuttaEulerAngles);

            final var error1 = Utils.normF(diffAngles1);
            final var error2 = Utils.normF(diffAngles2);
            final var error3 = Utils.normF(diffAngles3);
            final var error4 = Utils.normF(diffAngles4);
            final var error5 = Utils.normF(diffAngles5);
            final var error6 = Utils.normF(diffAngles6);

            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Suh error: %f radians", error1));
            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Trawny error: %f radians", error2));
            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Yuan error: %f radians", error3));
            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Euler error: %f radians", error4));
            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Mid-point error: %f radians", error5));
            Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                    String.format("Runge-Kutta error: %f radians", error6));

            avgError1 += error1;
            avgError2 += error2;
            avgError3 += error3;
            avgError4 += error4;
            avgError5 += error5;
            avgError6 += error6;
            num++;

            if (error1 < error2 || error1 < error3 || error1 < error4 || error1 < error5 || error1 < error6) {
                continue;
            }

            assertTrue(error1 >= error2);
            assertTrue(error1 >= error3);
            assertTrue(error1 >= error4);
            assertTrue(error1 >= error5);
            assertTrue(error1 >= error6);
            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // compute ranking of errors
        avgError1 /= num;
        avgError2 /= num;
        avgError3 /= num;
        avgError4 /= num;
        avgError5 /= num;
        avgError6 /= num;

        final var map = new HashMap<String, Double>();
        map.put("Suh", avgError1);
        map.put("Trawny", avgError2);
        map.put("Yuan", avgError3);
        map.put("Euler", avgError4);
        map.put("Mid-point", avgError5);
        map.put("Runga-Kutta", avgError6);

        final var methodsList = new ArrayList<>(map.entrySet()).stream()
                .sorted(Map.Entry.comparingByValue())
                .map(entry -> entry.getKey() + ": " + entry.getValue())
                .toList();
        final var methods = String.join(", ", methodsList);
        Logger.getLogger(SuhQuaternionStepIntegratorTest.class.getName()).log(Level.INFO,
                String.format("Methods (ordered more accurate to less accurate): %s", methods));
    }

    private static Quaternion getInitialAttitude() {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        return new Quaternion(roll, pitch, yaw);
    }
}
