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

import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;

/**
 * Class in charge of performing integration steps of rotations.
 * This implementation uses a Runge-Kutta integration algorithm to obtain
 * accurate results on {@link EasyGyroscopeCalibrator}
 */
public class QuaternionIntegrator {

    /**
     * Constructor.
     * Prevents instantiation of helper class.
     */
    private QuaternionIntegrator() {
        // no action needed
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at an initial attitude to obtain a final attitude.
     *
     * @param sequence        sequence of gyroscope measurements to be integrated.
     * @param initialAttitude (optional) initial attitude to be used. If null, then the
     *                        identity attitude will be used.
     * @param type            type of step integrator to be used.
     * @param result          resulting rotation after integration.
     * @throws RotationException if a numerical error occurs.
     */
    public static void integrateGyroSequence(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final Quaternion initialAttitude, final QuaternionStepIntegratorType type, final Quaternion result)
            throws RotationException {

        if (initialAttitude != null) {
            // if provided initial attitude
            result.fromRotation(initialAttitude);
        } else {
            // if no initial attitude is provided, we use the identity
            result.setA(1.0);
            result.setB(0.0);
            result.setC(0.0);
            result.setD(0.0);
        }


        final var sortedMeasurements = sequence.getSortedItems();
        final var stepIntegrator = QuaternionStepIntegrator.create(type);

        var first = true;
        var previousTimestamp = 0.0;
        var previousWx = 0.0;
        var previousWy = 0.0;
        var previousWz = 0.0;
        double currentWx;
        double currentWy;
        double currentWz;
        for (final var measurement : sortedMeasurements) {
            final var kinematics = measurement.getKinematics();

            currentWx = kinematics.getAngularRateX();
            currentWy = kinematics.getAngularRateY();
            currentWz = kinematics.getAngularRateZ();

            if (first) {
                previousTimestamp = measurement.getTimestampSeconds();

                // copy current angular rates to previous angular rates
                previousWx = currentWx;
                previousWy = currentWy;
                previousWz = currentWz;

                first = false;
                continue;
            }

            final var timestamp = measurement.getTimestampSeconds();


            final var dt = timestamp - previousTimestamp;

            stepIntegrator.integrate(result, previousWx, previousWy, previousWz, currentWx, currentWy, currentWz, dt,
                    result);

            // prepare data for next iteration

            // copy current angular rates to previous angular rates
            previousWx = currentWx;
            previousWy = currentWy;
            previousWz = currentWz;

            // copy timestamp to previous timestamp
            previousTimestamp = timestamp;
        }
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at the identity attitude to obtain a final attitude.
     *
     * @param sequence sequence of gyroscope measurements to be integrated.
     * @param type     type of step integrator to be used.
     * @param result   resulting rotation after integration.
     * @throws RotationException if a numerical error occurs.
     */
    public static void integrateGyroSequence(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final QuaternionStepIntegratorType type, final Quaternion result) throws RotationException {
        integrateGyroSequence(sequence, null, type, result);
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at an initial attitude to obtain a final attitude.
     *
     * @param sequence        sequence of gyroscope measurements to be integrated.
     * @param initialAttitude (optional) initial attitude to be used. If null, then the
     *                        identity attitude will be used.
     * @param type            type of step integrator to be used.
     * @return resulting rotation after integration.
     * @throws RotationException if a numerical error occurs.
     */
    public static Quaternion integrateGyroSequenceAndReturnNew(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final Quaternion initialAttitude, final QuaternionStepIntegratorType type) throws RotationException {
        final var result = new Quaternion();
        integrateGyroSequence(sequence, initialAttitude, type, result);
        return result;
    }

    /**
     * Integrates a sequence of gyroscope measurements contained within timed body kinematics,
     * starting at the identity attitude to obtain a final attitude.
     *
     * @param sequence sequence of gyroscope measurements to be integrated.
     * @param type     type of step integrator to be used.
     * @return resulting rotation after integration.
     * @throws RotationException if a numerical error occurs.
     */
    public static Quaternion integrateGyroSequenceAndReturnNew(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final QuaternionStepIntegratorType type) throws RotationException {
        final var result = new Quaternion();
        integrateGyroSequence(sequence, type, result);
        return result;
    }
}
