package frc.lib.util;

// NOTE: Could have used InterpolatingTreeMap ... :(

/**
 * Class for an calibration table.
 * Looks up a value from an ordered set of calibration values, interpolating
 * results as necessary.
 */
public class CalibrationTable<T extends Interpolatable<T>> {
    private CalibrationValue<T>[] table;

    public record CalibrationValue<T>(double index, T result) {}

    public CalibrationTable(CalibrationValue<T>[] table) {
        this.table = table;
    }

    public static double interpolate(double weight, double a, double b) {
        return weight * a + (1 - weight) * b;
    }

    public T lookup(double index) {
        CalibrationValue<T> priorEntry = null;
        T result = null;

        for (CalibrationValue<T> calibration : table) {
            if (index <= calibration.index) {
                if (priorEntry == null) {
                    // Anything closer that minimum calibration distance gets the same value as the
                    // first entry
                    result = calibration.result;
                } else {
                    // Linear interpolation between calibration entries
                    double fraction = (index - priorEntry.index) / (calibration.index - priorEntry.index);
                    result = calibration.result.interpolate(fraction, priorEntry.result);
                }

                break;
            }

            priorEntry = calibration;
        }

        // NOTE: Might be null if the calibration index has been exceeded
        return result;
    }
}