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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccumulatedBodyKinematicsNoiseEstimatorTest implements AccumulatedBodyKinematicsNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-12;
    private static final double LARGE_ABSOLUTE_ERROR = 0.1;

    private static final int N_SAMPLES = 100000;

    private static final int TIMES = 5;

    private int start;
    private int bodyKinematicsAdded;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator();

        // check default values
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());
        final var t2 = new Time(1.0, TimeUnit.NANOSECOND);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getAvgSpecificForceX(), 0.0);
        final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        assertEquals(0.0, avgFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);
        assertEquals(avgFx1, avgFx2);
        assertEquals(0.0, estimator.getAvgSpecificForceY(), 0.0);
        final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        assertEquals(0.0, avgFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);
        assertEquals(avgFy1, avgFy2);
        assertEquals(0.0, estimator.getAvgSpecificForceZ(), 0.0);
        final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        assertEquals(0.0, avgFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);
        assertEquals(avgFz1, avgFz2);
        final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(0.0, avgFTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final var avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(0.0, estimator.getAvgSpecificForceNorm(), 0.0);
        final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(0.0, avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);
        assertEquals(0.0, estimator.getAvgAngularRateX(), 0.0);
        final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(0.0, avgWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);
        assertEquals(0.0, estimator.getAvgAngularRateY(), 0.0);
        final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(0.0, avgWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);
        assertEquals(0.0, estimator.getAvgAngularRateZ(), 0.0);
        final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(0.0, avgWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);
        final var avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(0.0, avgWTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final var avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);
        assertEquals(0.0, estimator.getAvgAngularRateNorm(), 0.0);
        final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(0.0, avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);
        final var avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(0.0, avgKinematics1.getFx(), 0.0);
        assertEquals(0.0, avgKinematics1.getFy(), 0.0);
        assertEquals(0.0, avgKinematics1.getFz(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateZ(), 0.0);
        final var avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);
        assertEquals(0.0, estimator.getVarianceSpecificForceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceZ(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceX(), 0.0);
        final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(0.0, stdFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceY(), 0.0);
        final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(0.0, stdFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceZ(), 0.0);
        final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(0.0, stdFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);
        final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(0.0, stdFTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());
        final var stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceNorm(), 0.0);
        final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(0.0, stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationSpecificForce(), 0.0);
        final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(0.0, avgStdF1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(0.0, stdWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(0.0, stdWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(0.0, stdWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);
        final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(0.0, stdWTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final var stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);
        assertEquals(0.0, estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(0.0, stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationAngularSpeed(), 0.0);
        final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(0.0, avgStdW1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);
        final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(0.0, stdKinematics1.getFx(), 0.0);
        assertEquals(0.0, stdKinematics1.getFy(), 0.0);
        assertEquals(0.0, stdKinematics1.getFz(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateZ(), 0.0);
        final var stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);
        assertEquals(0.0, estimator.getSpecificForcePsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdZ(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgSpecificForceNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgAngularRateNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getAngularRateNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());
        final var t2 = new Time(1.0, TimeUnit.NANOSECOND);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getAvgSpecificForceX(), 0.0);
        final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        assertEquals(0.0, avgFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);
        assertEquals(avgFx1, avgFx2);
        assertEquals(0.0, estimator.getAvgSpecificForceY(), 0.0);
        final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        assertEquals(0.0, avgFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);
        assertEquals(avgFy1, avgFy2);
        assertEquals(0.0, estimator.getAvgSpecificForceZ(), 0.0);
        final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        assertEquals(0.0, avgFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);
        assertEquals(avgFz1, avgFz2);
        final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(0.0, avgFTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final var avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(0.0, estimator.getAvgSpecificForceNorm(), 0.0);
        final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(0.0, avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);
        assertEquals(0.0, estimator.getAvgAngularRateX(), 0.0);
        final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(0.0, avgWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);
        assertEquals(0.0, estimator.getAvgAngularRateY(), 0.0);
        final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(0.0, avgWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);
        assertEquals(0.0, estimator.getAvgAngularRateZ(), 0.0);
        final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(0.0, avgWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);
        final var avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(0.0, avgWTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final var avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);
        assertEquals(0.0, estimator.getAvgAngularRateNorm(), 0.0);
        final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(0.0, avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);
        final var avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(0.0, avgKinematics1.getFx(), 0.0);
        assertEquals(0.0, avgKinematics1.getFy(), 0.0);
        assertEquals(0.0, avgKinematics1.getFz(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateZ(), 0.0);
        final var avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);
        assertEquals(0.0, estimator.getVarianceSpecificForceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceZ(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceX(), 0.0);
        final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(0.0, stdFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceY(), 0.0);
        final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(0.0, stdFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceZ(), 0.0);
        final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(0.0, stdFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);
        final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(0.0, stdFTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());
        final var stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceNorm(), 0.0);
        final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(0.0, stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationSpecificForce(), 0.0);
        final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(0.0, avgStdF1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(0.0, stdWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(0.0, stdWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(0.0, stdWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);
        final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(0.0, stdWTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final var stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);
        assertEquals(0.0, estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(0.0, stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationAngularSpeed(), 0.0);
        final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(0.0, avgStdW1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);
        final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(0.0, stdKinematics1.getFx(), 0.0);
        assertEquals(0.0, stdKinematics1.getFy(), 0.0);
        assertEquals(0.0, stdKinematics1.getFz(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateZ(), 0.0);
        final var stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);
        assertEquals(0.0, estimator.getSpecificForcePsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdZ(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgSpecificForceNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgAngularRateNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getAngularRateNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);

        // set a new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var time2 = new Time(500, TimeUnit.MILLISECOND);
        estimator.setTimeInterval(time2);

        // check
        final var time3 = estimator.getTimeIntervalAsTime();
        final var time4 = new Time(0.0, TimeUnit.SECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertTrue(time2.equals(time3, ABSOLUTE_ERROR));
        assertTrue(time2.equals(time4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddBodyKinematicsAndReset1() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastKinematics = new BodyKinematics();

        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        var avgWx = 0.0;
        var avgWy = 0.0;
        var avgWz = 0.0;
        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        var varWx = 0.0;
        var varWy = 0.0;
        var varWz = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            final var fxi = kinematics.getFx();
            final var fyi = kinematics.getFy();
            final var fzi = kinematics.getFz();
            final var wxi = kinematics.getAngularRateX();
            final var wyi = kinematics.getAngularRateY();
            final var wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(fxi, fyi, fzi, wxi, wyi, wzi);

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            var diff = fxi - avgFx;
            var diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var avgFxB = estimator.getAvgSpecificForceX();
        final var avgFyB = estimator.getAvgSpecificForceY();
        final var avgFzB = estimator.getAvgSpecificForceZ();

        final var avgWxB = estimator.getAvgAngularRateX();
        final var avgWyB = estimator.getAvgAngularRateY();
        final var avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final var avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(), 0.0);
        final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(avgFNorm1.getValue().doubleValue(), estimator.getAvgSpecificForceNorm(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final var avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final var avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(), 0.0);
        final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(avgWNorm1.getValue().doubleValue(), estimator.getAvgAngularRateNorm(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final var avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(), SMALL_ABSOLUTE_ERROR);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);
        final var stdWx = Math.sqrt(varWx);
        final var stdWy = Math.sqrt(varWy);
        final var stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());

        final var stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(), estimator.getStandardDeviationSpecificForceNorm(), SMALL_ABSOLUTE_ERROR);
        final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final var avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(), SMALL_ABSOLUTE_ERROR);
        final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(), SMALL_ABSOLUTE_ERROR);
        final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(), SMALL_ABSOLUTE_ERROR);
        final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final var stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final var avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(), SMALL_ABSOLUTE_ERROR);
        final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final var psdFx = estimator.getSpecificForcePsdX();
        final var psdFy = estimator.getSpecificForcePsdY();
        final var psdFz = estimator.getSpecificForcePsdZ();
        final var psdWx = estimator.getAngularRatePsdX();
        final var psdWy = estimator.getAngularRatePsdY();
        final var psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final var rootPsdFx = Math.sqrt(psdFx);
        final var rootPsdFy = Math.sqrt(psdFy);
        final var rootPsdFz = Math.sqrt(psdFz);
        final var rootPsdWx = Math.sqrt(psdWx);
        final var rootPsdWy = Math.sqrt(psdWy);
        final var rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(), 0.0);

        final var avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final var avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final var normRootPsdF = Math.sqrt(rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy + rootPsdFz * rootPsdFz);
        final var normRootPsdW = Math.sqrt(rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy + rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getSpecificForceNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getAngularRateNoiseRootPsdNorm(), estimator.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final var accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(accelNoisePsd, estimator.getAvgSpecificForceNoisePsd(), ABSOLUTE_ERROR);

        final var gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(gyroNoisePsd, estimator.getAvgAngularRateNoisePsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Test
    void testAddBodyKinematicsAndReset2() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastKinematics = new BodyKinematics();

        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        var avgWx = 0.0;
        var avgWy = 0.0;
        var avgWz = 0.0;
        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        var varWx = 0.0;
        var varWy = 0.0;
        var varWz = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(lastKinematics, estimator.getLastBodyKinematics());
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            final var fxi = kinematics.getFx();
            final var fyi = kinematics.getFy();
            final var fzi = kinematics.getFz();
            final var wxi = kinematics.getAngularRateX();
            final var wyi = kinematics.getAngularRateY();
            final var wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(kinematics.getSpecificForceX(), kinematics.getSpecificForceY(),
                    kinematics.getSpecificForceZ(), kinematics.getAngularSpeedX(), kinematics.getAngularSpeedY(),
                    kinematics.getAngularSpeedZ());

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            var diff = fxi - avgFx;
            var diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var avgFxB = estimator.getAvgSpecificForceX();
        final var avgFyB = estimator.getAvgSpecificForceY();
        final var avgFzB = estimator.getAvgSpecificForceZ();

        final var avgWxB = estimator.getAvgAngularRateX();
        final var avgWyB = estimator.getAvgAngularRateY();
        final var avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final var avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(), 0.0);
        final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(avgFNorm1.getValue().doubleValue(), estimator.getAvgSpecificForceNorm(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final var avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final var avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(), 0.0);
        final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(estimator.getAvgAngularRateNorm(), avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final var avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(), SMALL_ABSOLUTE_ERROR);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);
        final var stdWx = Math.sqrt(varWx);
        final var stdWy = Math.sqrt(varWy);
        final var stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());

        final var stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(), estimator.getStandardDeviationSpecificForceNorm(), SMALL_ABSOLUTE_ERROR);
        final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final var avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(), SMALL_ABSOLUTE_ERROR);
        final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(), SMALL_ABSOLUTE_ERROR);
        final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(), SMALL_ABSOLUTE_ERROR);
        final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final var stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final var avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(), SMALL_ABSOLUTE_ERROR);
        final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final var psdFx = estimator.getSpecificForcePsdX();
        final var psdFy = estimator.getSpecificForcePsdY();
        final var psdFz = estimator.getSpecificForcePsdZ();
        final var psdWx = estimator.getAngularRatePsdX();
        final var psdWy = estimator.getAngularRatePsdY();
        final var psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final var rootPsdFx = Math.sqrt(psdFx);
        final var rootPsdFy = Math.sqrt(psdFy);
        final var rootPsdFz = Math.sqrt(psdFz);
        final var rootPsdWx = Math.sqrt(psdWx);
        final var rootPsdWy = Math.sqrt(psdWy);
        final var rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(), 0.0);

        final var avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final var avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final var normRootPsdF = Math.sqrt(rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy + rootPsdFz * rootPsdFz);
        final var normRootPsdW = Math.sqrt(rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy + rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getSpecificForceNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getAngularRateNoiseRootPsdNorm(), estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final var accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(estimator.getAvgSpecificForceNoisePsd(), accelNoisePsd, ABSOLUTE_ERROR);

        final var gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(estimator.getAvgAngularRateNoisePsd(), gyroNoisePsd, ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Test
    void testAddBodyKinematicsAndReset3() throws WrongSizeException, LockedException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateMg();
            final var gg = generateGg();
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
            final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
            final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

            final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

            final var estimator = new AccumulatedBodyKinematicsNoiseEstimator(this);

            reset();
            assertEquals(0, start);
            assertEquals(0, bodyKinematicsAdded);
            assertEquals(0, reset);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertFalse(estimator.isRunning());

            final var kinematics = new BodyKinematics();
            final var timeInterval = estimator.getTimeInterval();
            final var lastKinematics = new BodyKinematics();

            var avgFx = 0.0;
            var avgFy = 0.0;
            var avgFz = 0.0;
            var avgWx = 0.0;
            var avgWy = 0.0;
            var avgWz = 0.0;
            var varFx = 0.0;
            var varFy = 0.0;
            var varFz = 0.0;
            var varWx = 0.0;
            var varWy = 0.0;
            var varWz = 0.0;
            final var random = new Random();
            for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
                if (estimator.getLastBodyKinematics(lastKinematics)) {
                    assertEquals(lastKinematics, estimator.getLastBodyKinematics());
                    assertEquals(lastKinematics, kinematics);
                }

                BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

                final var fxi = kinematics.getFx();
                final var fyi = kinematics.getFy();
                final var fzi = kinematics.getFz();
                final var wxi = kinematics.getAngularRateX();
                final var wyi = kinematics.getAngularRateY();
                final var wzi = kinematics.getAngularRateZ();

                estimator.addBodyKinematics(kinematics.getSpecificForceTriad(), kinematics.getAngularRateTriad());

                assertTrue(estimator.getLastBodyKinematics(lastKinematics));
                assertEquals(lastKinematics, kinematics);
                assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
                assertFalse(estimator.isRunning());

                avgFx = avgFx * (double) i / (double) j + fxi / j;
                avgFy = avgFy * (double) i / (double) j + fyi / j;
                avgFz = avgFz * (double) i / (double) j + fzi / j;

                avgWx = avgWx * (double) i / (double) j + wxi / j;
                avgWy = avgWy * (double) i / (double) j + wyi / j;
                avgWz = avgWz * (double) i / (double) j + wzi / j;

                var diff = fxi - avgFx;
                var diff2 = diff * diff;
                varFx = varFx * (double) i / (double) j + diff2 / j;

                diff = fyi - avgFy;
                diff2 = diff * diff;
                varFy = varFy * (double) i / (double) j + diff2 / j;

                diff = fzi - avgFz;
                diff2 = diff * diff;
                varFz = varFz * (double) i / (double) j + diff2 / j;

                diff = wxi - avgWx;
                diff2 = diff * diff;
                varWx = varWx * (double) i / (double) j + diff2 / j;

                diff = wyi - avgWy;
                diff2 = diff * diff;
                varWy = varWy * (double) i / (double) j + diff2 / j;

                diff = wzi - avgWz;
                diff2 = diff * diff;
                varWz = varWz * (double) i / (double) j + diff2 / j;
            }

            assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
            assertEquals(1, start);
            assertEquals(N_SAMPLES, bodyKinematicsAdded);
            assertEquals(0, reset);

            final var avgFxB = estimator.getAvgSpecificForceX();
            final var avgFyB = estimator.getAvgSpecificForceY();
            final var avgFzB = estimator.getAvgSpecificForceZ();

            final var avgWxB = estimator.getAvgAngularRateX();
            final var avgWyB = estimator.getAvgAngularRateY();
            final var avgWzB = estimator.getAvgAngularRateZ();

            if (Math.abs(avgFx - avgFxB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgFy - avgFyB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgFz - avgFzB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWx - avgWxB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWy - avgWyB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWz - avgWzB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

            assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

            assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

            final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
            final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

            assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
            assertEquals(avgFx1, avgFx2);

            final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
            final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

            assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
            assertEquals(avgFy1, avgFy2);

            final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
            final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

            assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
            assertEquals(avgFz1, avgFz2);

            final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
            assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
            final var avgFTriad2 = new AccelerationTriad();
            estimator.getAvgSpecificForceAsTriad(avgFTriad2);
            assertEquals(avgFTriad1, avgFTriad2);
            assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(), 0.0);
            final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
            assertEquals(estimator.getAvgSpecificForceNorm(), avgFNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
            final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
            assertEquals(avgFNorm1, avgFNorm2);

            final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
            assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
            final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateXAsMeasurement(avgWx2);
            assertEquals(avgWx1, avgWx2);

            final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
            assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
            final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateYAsMeasurement(avgWy2);
            assertEquals(avgWy1, avgWy2);

            final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
            assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
            final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateZAsMeasurement(avgWz2);
            assertEquals(avgWz1, avgWz2);

            final var avgWTriad1 = estimator.getAvgAngularRateTriad();
            assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
            final var avgWTriad2 = new AngularSpeedTriad();
            estimator.getAvgAngularRateTriad(avgWTriad2);
            assertEquals(avgWTriad1, avgWTriad2);

            assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(), 0.0);
            final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
            assertEquals(avgWNorm1.getValue().doubleValue(), estimator.getAvgAngularRateNorm(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
            final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
            assertEquals(avgWNorm1, avgWNorm2);

            final var avgKinematics1 = estimator.getAvgBodyKinematics();
            assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWx, avgKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
            final var avgKinematics2 = new BodyKinematics();
            estimator.getAvgBodyKinematics(avgKinematics2);
            assertEquals(avgKinematics1, avgKinematics2);

            assertEquals(varFx, estimator.getVarianceSpecificForceX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(varFy, estimator.getVarianceSpecificForceY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(varFz, estimator.getVarianceSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(varWx, estimator.getVarianceAngularRateX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(varWy, estimator.getVarianceAngularRateY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(varWz, estimator.getVarianceAngularRateZ(), SMALL_ABSOLUTE_ERROR);

            final var stdFx = Math.sqrt(varFx);
            final var stdFy = Math.sqrt(varFy);
            final var stdFz = Math.sqrt(varFz);
            final var stdWx = Math.sqrt(varWx);
            final var stdWy = Math.sqrt(varWy);
            final var stdWz = Math.sqrt(varWz);

            assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(), SMALL_ABSOLUTE_ERROR);
            final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
            assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
            final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
            assertEquals(stdFx1, stdFx2);

            assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(), SMALL_ABSOLUTE_ERROR);
            final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
            assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
            final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
            assertEquals(stdFy1, stdFy2);

            assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
            final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
            assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
            final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
            assertEquals(stdFz1, stdFz2);

            final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
            assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());

            final var stdFTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
            assertEquals(stdFTriad1, stdFTriad2);

            assertEquals(stdFTriad1.getNorm(), estimator.getStandardDeviationSpecificForceNorm(), SMALL_ABSOLUTE_ERROR);
            final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
            assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
            final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
            assertEquals(stdFNorm1, stdFNorm2);

            final var avgStdF = (stdFx + stdFy + stdFz) / 3.0;
            assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(), SMALL_ABSOLUTE_ERROR);
            final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
            assertEquals(avgStdF, avgStdF1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
            final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
            assertEquals(avgStdF1, avgStdF2);

            assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(), SMALL_ABSOLUTE_ERROR);
            final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
            assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
            final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
            assertEquals(stdWx1, stdWx2);

            assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(), SMALL_ABSOLUTE_ERROR);
            final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
            assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
            final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
            assertEquals(stdWy1, stdWy2);

            assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(), SMALL_ABSOLUTE_ERROR);
            final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
            assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
            final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
            assertEquals(stdWz1, stdWz2);

            final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
            assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
            final var stdWTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
            assertEquals(stdWTriad1, stdWTriad2);

            assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
            final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
            assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
            final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
            assertEquals(stdWNorm1, stdWNorm2);

            final var avgStdW = (stdWx + stdWy + stdWz) / 3.0;
            assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(), SMALL_ABSOLUTE_ERROR);
            final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
            assertEquals(avgStdW, avgStdW1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
            final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
            assertEquals(avgStdW1, avgStdW2);

            final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
            assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
            final var stdKinematics2 = new BodyKinematics();
            estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
            assertEquals(stdKinematics1, stdKinematics2);

            final var psdFx = estimator.getSpecificForcePsdX();
            final var psdFy = estimator.getSpecificForcePsdY();
            final var psdFz = estimator.getSpecificForcePsdZ();
            final var psdWx = estimator.getAngularRatePsdX();
            final var psdWy = estimator.getAngularRatePsdY();
            final var psdWz = estimator.getAngularRatePsdZ();

            assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

            final var rootPsdFx = Math.sqrt(psdFx);
            final var rootPsdFy = Math.sqrt(psdFy);
            final var rootPsdFz = Math.sqrt(psdFz);
            final var rootPsdWx = Math.sqrt(psdWx);
            final var rootPsdWy = Math.sqrt(psdWy);
            final var rootPsdWz = Math.sqrt(psdWz);

            assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(), 0.0);
            assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(), 0.0);
            assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(), 0.0);
            assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(), 0.0);
            assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(), 0.0);
            assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(), 0.0);

            final var avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
            final var avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
            final var normRootPsdF = Math.sqrt(rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy + rootPsdFz * rootPsdFz);
            final var normRootPsdW = Math.sqrt(rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy + rootPsdWz * rootPsdWz);

            assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(), SMALL_ABSOLUTE_ERROR);
            assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
            assertEquals(estimator.getSpecificForceNoiseRootPsdNorm(),
                    estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
            assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(), SMALL_ABSOLUTE_ERROR);
            assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
            assertEquals(estimator.getAngularRateNoiseRootPsdNorm(), estimator.getGyroscopeBaseNoiseLevelRootPsd(),
                    0.0);

            assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

            assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

            final var accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
            assertEquals(accelNoisePsd, estimator.getAvgSpecificForceNoisePsd(), ABSOLUTE_ERROR);

            final var gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
            assertEquals(gyroNoisePsd, estimator.getAvgAngularRateNoisePsd(), ABSOLUTE_ERROR);

            // reset
            assertTrue(estimator.reset());

            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertFalse(estimator.isRunning());
            assertEquals(1, reset);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testAddBodyKinematicsAndReset4() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastKinematics = new BodyKinematics();

        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        var avgWx = 0.0;
        var avgWy = 0.0;
        var avgWz = 0.0;
        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        var varWx = 0.0;
        var varWy = 0.0;
        var varWz = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(lastKinematics, estimator.getLastBodyKinematics());
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            final var fxi = kinematics.getFx();
            final var fyi = kinematics.getFy();
            final var fzi = kinematics.getFz();
            final var wxi = kinematics.getAngularRateX();
            final var wyi = kinematics.getAngularRateY();
            final var wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(kinematics);

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            var diff = fxi - avgFx;
            var diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var avgFxB = estimator.getAvgSpecificForceX();
        final var avgFyB = estimator.getAvgSpecificForceY();
        final var avgFzB = estimator.getAvgSpecificForceZ();

        final var avgWxB = estimator.getAvgAngularRateX();
        final var avgWyB = estimator.getAvgAngularRateY();
        final var avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final var avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final var avgFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final var avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final var avgFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final var avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final var avgFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final var avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final var avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(), 0.0);
        final var avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(avgFNorm1.getValue().doubleValue(), estimator.getAvgSpecificForceNorm(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final var avgFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final var avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final var avgWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final var avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final var avgWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final var avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final var avgWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final var avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final var avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(), 0.0);
        final var avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(avgWNorm1.getValue().doubleValue(), estimator.getAvgAngularRateNorm(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final var avgWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final var avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(), SMALL_ABSOLUTE_ERROR);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);
        final var stdWx = Math.sqrt(varWx);
        final var stdWy = Math.sqrt(varWy);
        final var stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(), SMALL_ABSOLUTE_ERROR);
        final var stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final var stdFx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(), SMALL_ABSOLUTE_ERROR);
        final var stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final var stdFy2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(), SMALL_ABSOLUTE_ERROR);
        final var stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final var stdFz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final var stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());

        final var stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(), estimator.getStandardDeviationSpecificForceNorm(), SMALL_ABSOLUTE_ERROR);
        final var stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final var stdFNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final var avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(), SMALL_ABSOLUTE_ERROR);
        final var avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final var avgStdF2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(), SMALL_ABSOLUTE_ERROR);
        final var stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final var stdWx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(), SMALL_ABSOLUTE_ERROR);
        final var stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final var stdWy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final var stdWz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final var stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final var stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final var stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final var stdWNorm2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final var avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(), SMALL_ABSOLUTE_ERROR);
        final var avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final var avgStdW2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final var stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final var stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final var psdFx = estimator.getSpecificForcePsdX();
        final var psdFy = estimator.getSpecificForcePsdY();
        final var psdFz = estimator.getSpecificForcePsdZ();
        final var psdWx = estimator.getAngularRatePsdX();
        final var psdWy = estimator.getAngularRatePsdY();
        final var psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final var rootPsdFx = Math.sqrt(psdFx);
        final var rootPsdFy = Math.sqrt(psdFy);
        final var rootPsdFz = Math.sqrt(psdFz);
        final var rootPsdWx = Math.sqrt(psdWx);
        final var rootPsdWy = Math.sqrt(psdWy);
        final var rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(), 0.0);

        final var avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final var avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final var normRootPsdF = Math.sqrt(rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy + rootPsdFz * rootPsdFz);
        final var normRootPsdW = Math.sqrt(rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy + rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getSpecificForceNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(), SMALL_ABSOLUTE_ERROR);
        assertEquals(estimator.getAngularRateNoiseRootPsdNorm(), estimator.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final var accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(accelNoisePsd, estimator.getAvgSpecificForceNoisePsd(), ABSOLUTE_ERROR);

        final var gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(gyroNoisePsd, estimator.getAvgAngularRateNoisePsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Override
    public void onStart(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onBodyKinematicsAdded(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        bodyKinematicsAdded++;
    }

    @Override
    public void onReset(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        bodyKinematicsAdded = 0;
        reset = 0;
    }

    private void checkLocked(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(a, a, a, w, w, w));
        final var aTriad = new AccelerationTriad();
        final var wTriad = new AngularSpeedTriad();
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(aTriad, wTriad));
        final var kinematics = new BodyKinematics();
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(kinematics));
        assertThrows(LockedException.class, () -> assertFalse(estimator.reset()));
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMa() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
