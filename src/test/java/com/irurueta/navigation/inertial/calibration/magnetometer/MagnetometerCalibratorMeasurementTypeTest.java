package com.irurueta.navigation.inertial.calibration.magnetometer;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class MagnetometerCalibratorMeasurementTypeTest {

    @Test
    void testValues() {
        assertEquals(3, MagnetometerCalibratorMeasurementType.values().length);
    }
}
