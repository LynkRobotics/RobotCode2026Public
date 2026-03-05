package frc.lib.math;

public class Hysteresis {
    // Return one of two enums based of the comparison of a value to a threshold, with hysteresis to prevent rapid switching between the two enums
    public static <E extends Enum<E>> E calculate(double value, double threshold, double hysteresis, E currentEnum, E lowEnum, E highEnum) {
        if (value < threshold - hysteresis) {
            return lowEnum;
        } else if (value > threshold + hysteresis) {
            return highEnum;
        } else if (value < threshold) {
            if (currentEnum == highEnum) {
                return highEnum;
            } else {
                return lowEnum;
            }
        } else { // value >= threshold
            if (currentEnum == lowEnum) {
                return lowEnum;
            } else {
                return highEnum;
            }
        }
    }
}