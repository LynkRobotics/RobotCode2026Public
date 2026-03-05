package frc.lib.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.lib.util.CalibrationTable.CalibrationValue;

public class CalibrationTableTest {

    // Concrete implementation of Interpolatable for doubles
    public static class InterpolatableDouble implements Interpolatable<InterpolatableDouble> {
        private final double value;

        public InterpolatableDouble(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        @Override
        public InterpolatableDouble interpolate(double weight, InterpolatableDouble other) {
            return new InterpolatableDouble(CalibrationTable.interpolate(weight, this.value, other.value));
        }
    }

    @Test
    public void testLookup() {
        @SuppressWarnings("unchecked")
        CalibrationTable<InterpolatableDouble> calibrationTable = new CalibrationTable<>((CalibrationValue<InterpolatableDouble>[])
            new CalibrationValue<?>[] {
                new CalibrationValue<>(0.0, new InterpolatableDouble(10.0)),
                new CalibrationValue<>(5.0, new InterpolatableDouble(20.0)),
                new CalibrationValue<>(10.0, new InterpolatableDouble(40.0))
            }
        );

        // Test exact matches
        assertEquals(10.0, calibrationTable.lookup(0.0).getValue(), 1e-6);
        assertEquals(20.0, calibrationTable.lookup(5.0).getValue(), 1e-6);
        assertEquals(40.0, calibrationTable.lookup(10.0).getValue(), 1e-6);

        // Test interpolation
        assertEquals(15.0, calibrationTable.lookup(2.5).getValue(), 1e-6);
        assertEquals(30.0, calibrationTable.lookup(7.5).getValue(), 1e-6);

        // Test out-of-bounds
        assertNull(calibrationTable.lookup(15.0));
    }
}