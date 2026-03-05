package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.CalibrationTable;
import frc.lib.util.Interpolatable;
import frc.lib.util.LynkMotor.MotorConfig;
import frc.robot.Ports;

public class ShooterConstants {
    public static final double gearing = (18.0 / 18.0);
	public static final double idleRPM = 2000.0;
	public static final double slowRampTime = 1.5; // seconds to go from 0 to idle speed when "slow" mode is enabled
	public static final double acceptableDeltaRPM = 40.0;
	public static final double maxSpinUpTime = 0.30; // seconds
	public static final Voltage reverseVoltage = Units.Volts.of(-5.0); // for unjamming

	public enum ShooterMode {
		STOPPED,
		IDLE,
		SPINNING,
		PASSING,
		SHOOTING;
	}

    public static record ShooterSetpoint(double flywheelRPM, double kickerAngleDeg) implements Interpolatable<ShooterSetpoint> {
        @Override
        public ShooterSetpoint interpolate(double weight, ShooterSetpoint other) {
            return new ShooterSetpoint(
                CalibrationTable.interpolate(weight, flywheelRPM, other.flywheelRPM),
                CalibrationTable.interpolate(weight, kickerAngleDeg, other.kickerAngleDeg));
        }
    }

    @SuppressWarnings("unchecked")
    private static final CalibrationTable.CalibrationValue<ShooterSetpoint>[] shootingValues =
        (CalibrationTable.CalibrationValue<ShooterSetpoint>[]) new CalibrationTable.CalibrationValue[] {
			new CalibrationTable.CalibrationValue<>(1.70, new ShooterSetpoint(2350, 0)),
			new CalibrationTable.CalibrationValue<>(2.30, new ShooterSetpoint(2500, 0)),
			new CalibrationTable.CalibrationValue<>(3.06, new ShooterSetpoint(2700, 0)),
			new CalibrationTable.CalibrationValue<>(3.46, new ShooterSetpoint(2850, 0)),
			new CalibrationTable.CalibrationValue<>(4.405, new ShooterSetpoint(3150, 0)),
			new CalibrationTable.CalibrationValue<>(5.14, new ShooterSetpoint(3425, 0)),
			new CalibrationTable.CalibrationValue<>(5.68, new ShooterSetpoint(3650, 0)),
            new CalibrationTable.CalibrationValue<>(Double.POSITIVE_INFINITY, new ShooterSetpoint(3800, 0)),
        };
    public static final CalibrationTable<ShooterSetpoint> shootingTable = new CalibrationTable<>(shootingValues);

    @SuppressWarnings("unchecked")
    private static final CalibrationTable.CalibrationValue<ShooterSetpoint>[] passingValues =
        (CalibrationTable.CalibrationValue<ShooterSetpoint>[]) new CalibrationTable.CalibrationValue[] {
            new CalibrationTable.CalibrationValue<>(0.00, new ShooterSetpoint(1500, 0)),
            new CalibrationTable.CalibrationValue<>(1.00, new ShooterSetpoint(1500, 0)),
            new CalibrationTable.CalibrationValue<>(2.10, new ShooterSetpoint(2000, 0)),
            new CalibrationTable.CalibrationValue<>(4.13, new ShooterSetpoint(2800, 0)),
            new CalibrationTable.CalibrationValue<>(6.17, new ShooterSetpoint(3450, 0)),
            new CalibrationTable.CalibrationValue<>(7.70, new ShooterSetpoint(4600, 0)),
            new CalibrationTable.CalibrationValue<>(9.60, new ShooterSetpoint(5500, 0)),
            new CalibrationTable.CalibrationValue<>(Double.POSITIVE_INFINITY, new ShooterSetpoint(6000, 0)),
        };
    public static final CalibrationTable<ShooterSetpoint> passingTable = new CalibrationTable<>(passingValues);

	public static MotorConfig getFlywheelMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Flywheel A";
		config.port = Ports.FLYWHEEL_A;

		// Unloaded config -- TODO adjust for higher RPM values, too?
        config.talonConfig.Slot0.kP = 0.50; // 0.6 too much
        config.talonConfig.Slot0.kI = 0.0;
        config.talonConfig.Slot0.kD = 0.0;
        config.talonConfig.Slot0.kS = 0.25; // 0.25 V to overcome static friction, empirically determined
        config.talonConfig.Slot0.kG = 0.0;
        config.talonConfig.Slot0.kV = 1.0 / 8.35; // 1.0 V per 8.35 RPS; might run a little high at high RPS
        config.talonConfig.Slot0.kA = 0.0;

		// Loaded config
        config.talonConfig.Slot1.kP = 0.60;
        config.talonConfig.Slot1.kI = 0.0;
        config.talonConfig.Slot1.kD = 0.0;
        config.talonConfig.Slot1.kS = 0.73;  // Loaded requires more volts -- but much less now that floor imparts more energy
        config.talonConfig.Slot1.kV = 0.120;
        config.talonConfig.Slot1.kG = 0.0;
        config.talonConfig.Slot1.kA = 0.0;


		// Motion Magic is used only when we adjust to idle speed
		config.talonConfig.MotionMagic.MotionMagicAcceleration = idleRPM / 60.0 / slowRampTime;
		config.talonConfig.MotionMagic.MotionMagicJerk = config.talonConfig.MotionMagic.MotionMagicAcceleration / 0.5;

		config.talonConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
		config.talonConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		config.talonConfig.Voltage.PeakForwardVoltage = 12.0;
		config.talonConfig.Voltage.PeakReverseVoltage = -12.0;

		config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

		config.talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.talonConfig.Feedback.SensorToMechanismRatio = gearing;
		config.talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return config;
    }

	public static MotorConfig getKickerMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Kicker";
		config.port = Ports.KICKER;

        // config.talonConfig.Slot0.kP = 300.0;
		// config.talonConfig.Slot0.kD = 0.0;
		// config.talonConfig.Slot0.kS = 0.0;
		// config.talonConfig.Slot0.kG = 0.0;
		// config.talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		// config.talonConfig.MotionMagic.MotionMagicAcceleration = 1000;
		// config.talonConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
		// config.talonConfig.MotionMagic.MotionMagicJerk = 100;

		config.talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.talonConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

		config.talonConfig.Voltage.PeakForwardVoltage = 12.0;
		config.talonConfig.Voltage.PeakReverseVoltage = -12.0;

		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 	ClimberPosition.STOWED.angle.plus(maxExtension).in(Units.Rotations);

		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 	ClimberPosition.FULLY_STOWED.angle.in(Units.Rotations);

		config.talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.talonConfig.Feedback.SensorToMechanismRatio = gearing;
		config.talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return config;
    }
}