/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.estimators.RadiiOfCurvatureEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

/**
 * Ground-truth trajectory used by the tests of the Kalman filtered estimators.
 * A trajectory is a list of true ECEF frames sampled at a constant rate, along with the error-free
 * body kinematics (specific force and angular rate) an ideal IMU would measure between each pair of
 * consecutive frames.
 * Sample {@code i} of the trajectory corresponds to time {@code i * sampleInterval}; kinematics
 * {@code i} are the ones measured between frames {@code i} and {@code i + 1}.
 */
final class KalmanFilteredEstimatorTrajectory {

    /**
     * Sample interval of the ready-made trajectories, expressed in seconds (100 Hz IMU).
     */
    static final double SAMPLE_INTERVAL = 0.01;

    /**
     * Latitude where the ready-made trajectories start, expressed in degrees.
     * A mid latitude is used so that neither the Earth rotation nor the gravity model degenerate.
     */
    static final double START_LATITUDE_DEGREES = 41.0;

    /**
     * Longitude where the ready-made trajectories start, expressed in degrees.
     */
    static final double START_LONGITUDE_DEGREES = 2.0;

    /**
     * Height above the ellipsoid kept by the ready-made trajectories, expressed in meters.
     */
    static final double START_HEIGHT = 100.0;

    /**
     * Acceleration and deceleration magnitude of {@link #accelerateThenDecelerate(double, double)},
     * expressed in meters per squared second.
     */
    static final double ACCELERATION = 2.0;

    /**
     * Mean speed of {@link #turningAndAccelerating(double)}, expressed in meters per second.
     */
    static final double CRUISE_SPEED = 10.0;

    /**
     * Amplitude of the speed oscillation of {@link #turningAndAccelerating(double)}, expressed in
     * meters per second.
     */
    static final double SPEED_AMPLITUDE = 5.0;

    /**
     * Period of the speed oscillation of {@link #turningAndAccelerating(double)}, expressed in
     * seconds.
     */
    static final double SPEED_PERIOD = 20.0;

    /**
     * Amplitude of the heading oscillation of {@link #turningAndAccelerating(double)}, expressed in
     * radians.
     */
    static final double HEADING_AMPLITUDE = Math.PI / 2.0;

    /**
     * Period of the heading oscillation of {@link #turningAndAccelerating(double)}, expressed in
     * seconds.
     */
    static final double HEADING_PERIOD = 30.0;

    /**
     * Number of integration sub-steps per sample used to integrate latitude and longitude from the
     * speed and heading profiles, so that the true position is consistent with the true velocity
     * well below the tolerances used by the tests.
     */
    private static final int SUBSTEPS = 10;

    /**
     * Factor to obtain the middle of an integration sub-step.
     */
    private static final double HALF = 0.5;

    /**
     * Sample interval expressed in seconds.
     */
    private final double sampleInterval;

    /**
     * True ECEF frames at each sample time.
     */
    private final List<ECEFFrame> frames;

    /**
     * Error-free body kinematics between each pair of consecutive frames.
     */
    private final List<BodyKinematics> kinematics;

    /**
     * Constructor.
     *
     * @param sampleInterval sample interval expressed in seconds.
     * @param nedFrames      true NED frames at each sample time.
     */
    private KalmanFilteredEstimatorTrajectory(final double sampleInterval, final List<NEDFrame> nedFrames) {
        this.sampleInterval = sampleInterval;

        final var ecefFrames = new ArrayList<ECEFFrame>();
        for (final var nedFrame : nedFrames) {
            ecefFrames.add(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame));
        }

        final var trueKinematics = new ArrayList<BodyKinematics>();
        for (var i = 1; i < ecefFrames.size(); i++) {
            trueKinematics.add(ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(sampleInterval,
                    ecefFrames.get(i), ecefFrames.get(i - 1)));
        }

        frames = List.copyOf(ecefFrames);
        kinematics = List.copyOf(trueKinematics);
    }

    /**
     * Creates a trajectory from a function returning the true NED frame at a given time.
     *
     * @param sampleInterval sample interval expressed in seconds.
     * @param duration       duration of the trajectory expressed in seconds.
     * @param trueFrame      function returning the true NED frame for a time expressed in seconds
     *                       since the start of the trajectory.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory fromFunction(
            final double sampleInterval, final double duration, final DoubleFunction<NEDFrame> trueFrame) {
        final var numSamples = Math.round(duration / sampleInterval);
        final var nedFrames = new ArrayList<NEDFrame>();
        for (var i = 0; i <= numSamples; i++) {
            nedFrames.add(trueFrame.apply(i * sampleInterval));
        }
        return new KalmanFilteredEstimatorTrajectory(sampleInterval, nedFrames);
    }

    /**
     * Creates a level trajectory at constant height from a speed and a heading profile.
     * The body keeps zero roll and pitch and its yaw equals the heading, the velocity points along
     * the heading, and latitude and longitude are obtained by integrating the velocity so that the
     * true position and the true velocity are consistent.
     *
     * @param initialPosition position at the start of the trajectory.
     * @param sampleInterval  sample interval expressed in seconds.
     * @param duration        duration of the trajectory expressed in seconds.
     * @param speed           function returning the horizontal speed expressed in meters per second
     *                        for a time expressed in seconds.
     * @param heading         function returning the heading expressed in radians for a time
     *                        expressed in seconds.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory fromSpeedAndHeading(
            final NEDPosition initialPosition, final double sampleInterval, final double duration,
            final DoubleUnaryOperator speed, final DoubleUnaryOperator heading) {
        final var numSamples = Math.round(duration / sampleInterval);
        final var height = initialPosition.getHeight();
        final var step = sampleInterval / SUBSTEPS;

        final var nedFrames = new ArrayList<NEDFrame>();
        var latitude = initialPosition.getLatitude();
        var longitude = initialPosition.getLongitude();
        nedFrames.add(levelFrame(latitude, longitude, height, speed.applyAsDouble(0.0),
                heading.applyAsDouble(0.0)));

        for (var i = 0; i < numSamples; i++) {
            for (var j = 0; j < SUBSTEPS; j++) {
                // explicit midpoint (second order Runge-Kutta) integration of latitude and longitude
                final var time = i * sampleInterval + j * step;
                final var middleTime = time + HALF * step;
                final var middleLatitude = latitude + HALF * step * latitudeRate(latitude, height,
                        speed.applyAsDouble(time), heading.applyAsDouble(time));
                final var middleSpeed = speed.applyAsDouble(middleTime);
                final var middleHeading = heading.applyAsDouble(middleTime);

                latitude += step * latitudeRate(middleLatitude, height, middleSpeed, middleHeading);
                longitude += step * longitudeRate(middleLatitude, height, middleSpeed, middleHeading);
            }

            final var time = (i + 1) * sampleInterval;
            nedFrames.add(levelFrame(latitude, longitude, height, speed.applyAsDouble(time),
                    heading.applyAsDouble(time)));
        }

        return new KalmanFilteredEstimatorTrajectory(sampleInterval, nedFrames);
    }

    /**
     * Creates a 100 Hz trajectory of a body that stays at the same position while yawing at a
     * constant rate, starting with zero yaw.
     *
     * @param yawRate  yaw rate expressed in radians per second.
     * @param duration duration expressed in seconds.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory yawingStationary(final double yawRate, final double duration) {
        return fromSpeedAndHeading(startPosition(), SAMPLE_INTERVAL, duration, time -> 0.0,
                time -> yawRate * time);
    }

    /**
     * Creates a 100 Hz trajectory of a body moving north at a constant speed and height.
     *
     * @param speed    speed expressed in meters per second.
     * @param duration duration expressed in seconds.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory northAtConstantSpeed(final double speed, final double duration) {
        return fromSpeedAndHeading(startPosition(), SAMPLE_INTERVAL, duration, time -> speed, time -> 0.0);
    }

    /**
     * Creates a 100 Hz trajectory of a body that starts at rest, accelerates north at
     * {@link #ACCELERATION} and then decelerates at a constant rate until it is at rest again.
     *
     * @param accelerationDuration duration of the acceleration expressed in seconds.
     * @param decelerationDuration duration of the deceleration expressed in seconds.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory accelerateThenDecelerate(
            final double accelerationDuration, final double decelerationDuration) {
        final var maxSpeed = ACCELERATION * accelerationDuration;
        return fromSpeedAndHeading(startPosition(), SAMPLE_INTERVAL,
                accelerationDuration + decelerationDuration,
                time -> time <= accelerationDuration
                        ? ACCELERATION * time
                        : Math.max(0.0, maxSpeed * (1.0 - (time - accelerationDuration) / decelerationDuration)),
                time -> 0.0);
    }

    /**
     * Creates a 100 Hz trajectory of a body whose speed and heading both change continuously
     * (sinusoidal speed around {@link #CRUISE_SPEED} and sinusoidal heading).
     *
     * @param duration duration expressed in seconds.
     * @return a new trajectory.
     */
    static KalmanFilteredEstimatorTrajectory turningAndAccelerating(final double duration) {
        return fromSpeedAndHeading(startPosition(), SAMPLE_INTERVAL, duration,
                time -> CRUISE_SPEED + SPEED_AMPLITUDE * Math.sin(2.0 * Math.PI * time / SPEED_PERIOD),
                time -> HEADING_AMPLITUDE * Math.sin(2.0 * Math.PI * time / HEADING_PERIOD));
    }

    /**
     * Navigates from an initial frame by chaining one call to
     * {@link ECEFInertialNavigator#navigateECEF(double, ECEFFrame, BodyKinematics, ECEFFrame)} per
     * kinematics sample.
     * This is the oracle of the tests checking that an estimator integrates every IMU sample.
     *
     * @param initialFrame   frame at the start. It is not modified.
     * @param kinematicsList kinematics measured during each consecutive sample interval.
     * @param sampleInterval sample interval expressed in seconds.
     * @return frame obtained after integrating all the samples.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    static ECEFFrame chainNavigator(
            final ECEFFrame initialFrame, final List<BodyKinematics> kinematicsList, final double sampleInterval)
            throws InertialNavigatorException {
        final var result = new ECEFFrame(initialFrame);
        for (final var bodyKinematics : kinematicsList) {
            ECEFInertialNavigator.navigateECEF(sampleInterval, result, bodyKinematics, result);
        }
        return result;
    }

    /**
     * Gets sample interval expressed in seconds.
     *
     * @return sample interval.
     */
    double getSampleInterval() {
        return sampleInterval;
    }

    /**
     * Gets the true ECEF frames at each sample time. Frame {@code i} corresponds to time
     * {@code i * sampleInterval}.
     *
     * @return unmodifiable list of true frames.
     */
    List<ECEFFrame> getFrames() {
        return frames;
    }

    /**
     * Gets the error-free body kinematics between consecutive frames. Kinematics {@code i} are the
     * ones measured between frames {@code i} and {@code i + 1}.
     *
     * @return unmodifiable list of true kinematics.
     */
    List<BodyKinematics> getKinematics() {
        return kinematics;
    }

    /**
     * Gets a copy of the true frame at the start of the trajectory.
     *
     * @return copy of the first true frame.
     */
    ECEFFrame getInitialFrame() {
        return new ECEFFrame(frames.getFirst());
    }

    /**
     * Gets a copy of the true frame at the end of the trajectory.
     *
     * @return copy of the last true frame.
     */
    ECEFFrame getFinalFrame() {
        return new ECEFFrame(frames.getLast());
    }

    /**
     * Gets the time of a sample expressed in seconds since the start of the trajectory.
     *
     * @param index index of the sample (frame).
     * @return time of the sample.
     */
    double getTimestamp(final int index) {
        return index * sampleInterval;
    }

    /**
     * Gets the position where the ready-made trajectories start.
     *
     * @return start position.
     */
    private static NEDPosition startPosition() {
        return new NEDPosition(Math.toRadians(START_LATITUDE_DEGREES), Math.toRadians(START_LONGITUDE_DEGREES),
                START_HEIGHT);
    }

    /**
     * Rate of change of latitude for a level motion.
     *
     * @param latitude latitude expressed in radians.
     * @param height   height expressed in meters.
     * @param speed    horizontal speed expressed in meters per second.
     * @param heading  heading expressed in radians.
     * @return latitude rate expressed in radians per second.
     */
    private static double latitudeRate(
            final double latitude, final double height, final double speed, final double heading) {
        final var radii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(latitude);
        return speed * Math.cos(heading) / (radii.getRn() + height);
    }

    /**
     * Rate of change of longitude for a level motion.
     *
     * @param latitude latitude expressed in radians.
     * @param height   height expressed in meters.
     * @param speed    horizontal speed expressed in meters per second.
     * @param heading  heading expressed in radians.
     * @return longitude rate expressed in radians per second.
     */
    private static double longitudeRate(
            final double latitude, final double height, final double speed, final double heading) {
        final var radii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(latitude);
        return speed * Math.sin(heading) / ((radii.getRe() + height) * Math.cos(latitude));
    }

    /**
     * Builds the true NED frame of a level body whose yaw equals its heading.
     *
     * @param latitude  latitude expressed in radians.
     * @param longitude longitude expressed in radians.
     * @param height    height expressed in meters.
     * @param speed     horizontal speed expressed in meters per second.
     * @param heading   heading (and yaw) expressed in radians.
     * @return true NED frame.
     */
    private static NEDFrame levelFrame(
            final double latitude, final double longitude, final double height, final double speed,
            final double heading) {
        final var c = new CoordinateTransformation(0.0, 0.0, heading, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        try {
            return new NEDFrame(latitude, longitude, height, speed * Math.cos(heading), speed * Math.sin(heading),
                    0.0, c);
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            // never happens: transformation is always body to local navigation
            throw new IllegalStateException(e);
        }
    }
}
