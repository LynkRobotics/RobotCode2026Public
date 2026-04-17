package frc.robot;

import java.util.Optional;
import java.util.function.DoublePredicate;
import java.util.function.DoubleUnaryOperator;

import static frc.robot.Options.optHubActive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Encapsulate logic relating to 2026 REBUILT Shifts
public enum Shift {
    DISABLED("Disabled", t -> !DriverStation.isEnabled(), t -> -1),
    AUTO("Auto", t -> DriverStation.isAutonomous(), t -> t),
    TRANSITION("Transition", t -> t > 130, t -> t - 130),
    SHIFT_1("Shift 1", t -> t > 105, t -> t - 105),
    SHIFT_2("Shift 2", t -> t > 80, t -> t - 80),
    SHIFT_3("Shift 3", t -> t > 55, t -> t - 55),
    SHIFT_4("Shift 4", t -> t > 30, t -> t - 30),
    END_GAME("End Game", t -> t >= 0, t -> t),
    TESTING("Testing", t -> true, t -> -1);

    private static final Shift[] values = values();
    private static final int size = values.length;

    private final String niceName;
    private final DoublePredicate conditionFn;
    private final DoubleUnaryOperator timeLeftFn;

    private static Optional<Boolean> wonAuto = Optional.empty();

    private Shift(String name, DoublePredicate condition, DoubleUnaryOperator timeLeft) {
        niceName = name;
        conditionFn = condition;
        timeLeftFn = timeLeft;
    }

    public Shift next() {
        return values[(ordinal() + 1) % size];
    }

    @Override
    public String toString() {
        return niceName;
    }

    public static Shift lookup(double matchTime) {
        for (Shift shift : values) {
            if (shift.conditionFn.test(matchTime)) {
                return shift;
            }
        }
        assert false : "Failed to match time " + String.format("%1.1f", matchTime) + " to Shift";
        return Shift.DISABLED;
    }

    public double timeLeft(double matchTime) {
        return timeLeftFn.applyAsDouble(matchTime);
    }

    public static Optional<Boolean> wonAuto() {
        if (wonAuto.isEmpty()) {
            String gameData = DriverStation.getGameSpecificMessage();

            if (!gameData.isEmpty()) {
                wonAuto = switch (gameData.charAt(0)) {
                    case 'R' -> Optional.of(Robot.isRed());
                    case 'B' -> Optional.of(!Robot.isRed());
                    default -> Optional.empty(); // Invalid data
                };
                if (wonAuto.isPresent()) {
                    SmartDashboard.putString("Won Auto", wonAuto.get() ? "W" : "L");
                }
            }
        }

        return wonAuto;
    }

    public boolean isActive() {
        if (this == DISABLED) {
            return false;
        } else if (this == AUTO || this == TRANSITION || this == END_GAME) {
            return true;
        } else if (this == TESTING) {
            return optHubActive.get();
        }

        Optional<Boolean> wonAutoOpt = wonAuto();
        if (wonAutoOpt.isEmpty()) {
            return false;
        }

        boolean wonAuto = wonAutoOpt.get();
        if ((wonAuto && (this == SHIFT_2 || this == SHIFT_4)) || (!wonAuto && (this == SHIFT_1 || this == SHIFT_3))) {
            return true;
        }
        return false;
    }

    public boolean disabledNext() {
        return (this == AUTO || this == END_GAME);
    }

    public boolean isNextActive() {
        return disabledNext() ? false : next().isActive();
    }

    public static void updateDashboard() {
        updateDashboard(DriverStation.getMatchTime());
    }

    public static void updateDashboard(double matchTime) {
        Shift shift = Shift.lookup(matchTime);
        SmartDashboard.putString("Shift", shift.toString());
        SmartDashboard.putNumber("Shift Time", shift.timeLeft(matchTime));
        SmartDashboard.putBoolean("Hub Active", shift.isActive());
    }
}