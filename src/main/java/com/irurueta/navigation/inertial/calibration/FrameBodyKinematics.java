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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Contains a body kinematics measurement (accelerometer + gyroscope) along with
 * the corresponding frame (position, orientation and velocity) where the
 * kinematics measurement was made.
 */
public class FrameBodyKinematics implements Serializable, Cloneable {

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Current body kinematics measurement. Contains accelerometer and gyroscope measurements.
     */
    private BodyKinematics kinematics;

    /**
     * Contains current body position, velocity (which will typically be zero) and orientation
     * resolved around ECEF axes.
     */
    private ECEFFrame frame;

    /**
     * Contains body position, velocity (which will typically be zero) and orientation
     * resolved around ECEF axes of previous IMU measurement.
     */
    private ECEFFrame previousFrame;

    /**
     * Time interval expressed in seconds (s) between IMU measurements used to obtain
     * current frame and previous frame.
     */
    private double timeInterval;

    /**
     * Constructor.
     */
    public FrameBodyKinematics() {
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Constructor.
     *
     * @param frame current ECEF frame associated to measurement.
     */
    public FrameBodyKinematics(final ECEFFrame frame) {
        this.frame = frame;
    }

    /**
     * Constructor.
     *
     * @param frame current NED frame associated to measurement. Internally it will be
     *              converted to its corresponding ECEF frame.
     */
    public FrameBodyKinematics(final NEDFrame frame) {
        setNedFrame(frame);
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval expressed in seconds (s) between IMU measurements
     *                     used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final double timeInterval) {
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param timeInterval time interval between IMU measurements used to obtain
     *                     current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final Time timeInterval) {
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     */
    public FrameBodyKinematics(final ECEFFrame frame, final ECEFFrame previousFrame) {
        this(frame);
        this.previousFrame = previousFrame;
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     */
    public FrameBodyKinematics(final NEDFrame frame, final NEDFrame previousFrame) {
        this(frame);
        setPreviousNedFrame(previousFrame);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final ECEFFrame frame, final ECEFFrame previousFrame, final double timeInterval) {
        this(frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final ECEFFrame frame, final ECEFFrame previousFrame, final Time timeInterval) {
        this(frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final NEDFrame frame, final NEDFrame previousFrame, final double timeInterval) {
        this(frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final NEDFrame frame, final NEDFrame previousFrame, final Time timeInterval) {
        this(frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     * @param frame      ECEF frame associated to measurement.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final ECEFFrame frame) {
        this(kinematics);
        this.frame = frame;
    }

    /**
     * Constructor.
     *
     * @param kinematics current body kinematics measurement.
     * @param frame      NED frame associated to measurement. Internally it will be
     *                   converted to its corresponding ECEF frame.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final NEDFrame frame) {
        this(kinematics);
        setNedFrame(frame);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final ECEFFrame frame, final ECEFFrame previousFrame) {
        this(kinematics, frame);
        this.previousFrame = previousFrame;
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final NEDFrame frame, final NEDFrame previousFrame) {
        this(kinematics, frame);
        setPreviousNedFrame(previousFrame);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final ECEFFrame frame, final ECEFFrame previousFrame,
                               final double timeInterval) {
        this(kinematics, frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current ECEF frame associated to measurement.
     * @param previousFrame previous ECEF frame associated to measurement.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final ECEFFrame frame, final ECEFFrame previousFrame,
                               final Time timeInterval) {
        this(kinematics, frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval expressed in seconds (s) between IMU measurements
     *                      used to obtain current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final NEDFrame frame, final NEDFrame previousFrame,
                               final double timeInterval) {
        this(kinematics, frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param kinematics    current body kinematics measurement.
     * @param frame         current NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param previousFrame previous NED frame associated to measurement. Internally it
     *                      will be converted to its corresponding ECEF frame.
     * @param timeInterval  time interval between IMU measurements used to obtain
     *                      current frame and previous frame.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public FrameBodyKinematics(final BodyKinematics kinematics, final NEDFrame frame, final NEDFrame previousFrame,
                               final Time timeInterval) {
        this(kinematics, frame, previousFrame);
        setTimeInterval(timeInterval);
    }

    /**
     * Constructor.
     *
     * @param input instance to copy data from.
     */
    public FrameBodyKinematics(final FrameBodyKinematics input) {
        copyFrom(input);
    }

    /**
     * Gets current body kinematics measurement. Contains accelerometer and gyroscope
     * measurements.
     *
     * @return current body kinematics measurement.
     */
    public BodyKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Sets current body kinematics measurement. Contains accelerometer and gyroscope
     * measurements.
     *
     * @param kinematics current body kinematics measurement to be set.
     */
    public void setKinematics(final BodyKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around ECEF axes associated to body kinematics measurement.
     *
     * @return current ECEF frame associated to body kinematics measurement or null if
     * not available.
     */
    public ECEFFrame getFrame() {
        return frame;
    }

    /**
     * Sets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around ECEF axes associated to body kinematics measurement.
     *
     * @param frame current ECEF frame
     */
    public void setFrame(final ECEFFrame frame) {
        this.frame = frame;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to body kinematics measurement.
     *
     * @return current NED frame associated to body kinematics measurement or null if
     * not available.
     */
    public NEDFrame getNedFrame() {
        return frame != null ? ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(frame) : null;
    }

    /**
     * Gets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to body kinematics measurement.
     *
     * @param result instance where result data will be stored if available.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getNedFrame(final NEDFrame result) {
        if (frame != null) {
            ECEFtoNEDFrameConverter.convertECEFtoNED(frame, result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets current body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to body kinematics measurement.
     * <p>
     * This method will internally store the corresponding ECEF frame to provided
     * NED frame value.
     *
     * @param nedFrame current NED frame associated to body kinematics measurement
     *                 to be set.
     */
    public void setNedFrame(final NEDFrame nedFrame) {
        if (nedFrame != null) {
            if (frame != null) {
                NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, frame);
            } else {
                frame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            }
        } else {
            frame = null;
        }
    }

    /**
     * Gets previous body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around ECEF axes associated to previous body kinematics measurement.
     *
     * @return previous ECEF frame associated to previous body kinematics measurement or
     * null if not available.
     */
    public ECEFFrame getPreviousFrame() {
        return previousFrame;
    }

    /**
     * Sets previous body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around ECEF axes associated to previous body kinematics measurement.
     *
     * @param previousFrame previous ECEF frame associated to previous body kinematics
     *                      measurement.
     */
    public void setPreviousFrame(final ECEFFrame previousFrame) {
        this.previousFrame = previousFrame;
    }

    /**
     * Gets previous body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to previous body kinematics measurement.
     *
     * @return previous NED frame associated to previous body kinematics measurement or
     * null if not available.
     */
    public NEDFrame getPreviousNedFrame() {
        return previousFrame != null ? ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(previousFrame) : null;
    }

    /**
     * Gets previous body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to previous body kinematics measurement.
     *
     * @param result instance where result data will be stored if available.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getPreviousNedFrame(final NEDFrame result) {
        if (previousFrame != null) {
            ECEFtoNEDFrameConverter.convertECEFtoNED(previousFrame, result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets previous body position (which will typically remain constant),
     * velocity (which will typically be zero) and orientation (which usually changes
     * with each measurement to perform calibration of a single device) resolved
     * around NED axes associated to previous body kinematics measurement.
     *
     * @param previousNedFrame previous NED frame associated to body kinematics
     *                         measurement to be set.
     */
    public void setPreviousNedFrame(final NEDFrame previousNedFrame) {
        if (previousNedFrame != null) {
            if (previousFrame != null) {
                NEDtoECEFFrameConverter.convertNEDtoECEF(previousNedFrame, previousFrame);
            } else {
                previousFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);
            }
        } else {
            previousFrame = null;
        }
    }

    /**
     * Gets time interval expressed in seconds (s) between IMU measurements used to
     * obtain current frame and previous frame.
     *
     * @return time interval expressed in seconds (s) between IMU measurements.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets time interval expressed in seconds (s) between IMU measurements used to
     * obtain current frame and previous frame.
     *
     * @param timeInterval time interval expressed in seconds (s) between IMU
     *                     measurements.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final double timeInterval) {
        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        this.timeInterval = timeInterval;
    }

    /**
     * Gets time interval between IMU measurements used to obtain current frame and
     * previous frame.
     *
     * @return time interval between IMU measurements.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between IMU measurements used to obtain current frame and
     * previous frame.
     *
     * @param result instance where result will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between IMU measurements used to obtain current frame and
     * previous frame.
     *
     * @param timeInterval time interval between IMU measurements to be set.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final Time timeInterval) {
        setTimeInterval(TimeConverter.convert(
                timeInterval.getValue().doubleValue(), timeInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy dara from.
     */
    public void copyFrom(final FrameBodyKinematics input) {
        if (input.kinematics != null) {
            if (kinematics == null) {
                kinematics = new BodyKinematics(input.kinematics);
            } else {
                kinematics.copyFrom(input.kinematics);
            }
        } else {
            kinematics = null;
        }

        if (input.frame != null) {
            if (frame == null) {
                frame = new ECEFFrame(input.frame);
            } else {
                frame.copyFrom(input.frame);
            }
        } else {
            frame = null;
        }

        if (input.previousFrame != null) {
            if (previousFrame == null) {
                previousFrame = new ECEFFrame(input.previousFrame);
            } else {
                previousFrame.copyFrom(input.previousFrame);
            }
        } else {
            previousFrame = null;
        }

        timeInterval = input.timeInterval;
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final FrameBodyKinematics output) {
        output.copyFrom(this);
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(kinematics, frame, previousFrame, timeInterval);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final FrameBodyKinematics other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum allowed difference between kinematics and frame values.
     * @return true if both instances are considered to be equal (up to provided
     * threshold), false otherwise.
     */
    public boolean equals(final FrameBodyKinematics other, final double threshold) {
        if (other == null) {
            return false;
        }

        return ((other.kinematics == null && kinematics == null)
                || (kinematics != null && kinematics.equals(other.kinematics, threshold)))
                && ((other.frame == null && frame == null)
                || (frame != null && frame.equals(other.frame, threshold)))
                && ((other.previousFrame == null && previousFrame == null)
                || (previousFrame != null && previousFrame.equals(other.previousFrame, threshold)))
                && Math.abs(other.timeInterval - timeInterval) <= threshold;
    }

    /**
     * Checks if provided object is a FrameBodyKinematics instance having exactly the same
     * contents as this instance.
     *
     * @param obj object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final var other = (FrameBodyKinematics) obj;
        return equals(other);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (FrameBodyKinematics) super.clone();
        copyTo(result);
        return result;
    }
}
