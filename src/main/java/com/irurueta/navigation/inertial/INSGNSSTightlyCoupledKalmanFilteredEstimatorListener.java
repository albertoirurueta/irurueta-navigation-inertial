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
package com.irurueta.navigation.inertial;

/**
 * Listener defining events of INSGNSSTightlyCoupledKalmanFilteredEstimatorListener.
 */
public interface INSGNSSTightlyCoupledKalmanFilteredEstimatorListener {

    /**
     * Called when GNSS measurements update starts.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateGNSSMeasurementsStart(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when GNSS measurements update ends.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateGNSSMeasurementsEnd(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when inertial INS measurements update starts.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateBodyKinematicsStart(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when inertial INS measurements update ends.
     *
     * @param estimator estimator raising the event.
     */
    void onUpdateBodyKinematicsEnd(
            final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation starts.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateStart(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when Kalman filter propagation ends.
     *
     * @param estimator estimator raising the event.
     */
    void onPropagateEnd(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);

    /**
     * Called when estimator is reset.
     *
     * @param estimator estimator raising the event.
     */
    void onReset(final INSGNSSTightlyCoupledKalmanFilteredEstimator estimator);
}
