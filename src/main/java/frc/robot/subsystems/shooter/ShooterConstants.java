package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
	public static final double loadedExpiry = 5.0; // seconds of shooting to consider the shooter no longer "loaded" because fuel is slower

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
			new CalibrationTable.CalibrationValue<>(1.62, new ShooterSetpoint(2250, 0)),
			new CalibrationTable.CalibrationValue<>(2.01, new ShooterSetpoint(2300, 0)),
			new CalibrationTable.CalibrationValue<>(2.50, new ShooterSetpoint(2450, 0)),
			new CalibrationTable.CalibrationValue<>(3.00, new ShooterSetpoint(2660, 0)),
			new CalibrationTable.CalibrationValue<>(3.50, new ShooterSetpoint(2795, 0)),
			new CalibrationTable.CalibrationValue<>(4.00, new ShooterSetpoint(2935, 0)),
			new CalibrationTable.CalibrationValue<>(4.50, new ShooterSetpoint(3100, 0)),
			new CalibrationTable.CalibrationValue<>(5.00, new ShooterSetpoint(3315, 0)),
			new CalibrationTable.CalibrationValue<>(5.50, new ShooterSetpoint(3550, 0)),
            new CalibrationTable.CalibrationValue<>(Double.POSITIVE_INFINITY, new ShooterSetpoint(3550, 0)),
        };
    public static final CalibrationTable<ShooterSetpoint> shootingTable = new CalibrationTable<>(shootingValues);

    @SuppressWarnings("unchecked")
    private static final CalibrationTable.CalibrationValue<ShooterSetpoint>[] passingValues =
        (CalibrationTable.CalibrationValue<ShooterSetpoint>[]) new CalibrationTable.CalibrationValue[] {
            new CalibrationTable.CalibrationValue<>(0.00, new ShooterSetpoint(2000, 0)),
            new CalibrationTable.CalibrationValue<>(2.44, new ShooterSetpoint(2000, 0)),
            new CalibrationTable.CalibrationValue<>(3.60, new ShooterSetpoint(2400, 0)),
            new CalibrationTable.CalibrationValue<>(4.50, new ShooterSetpoint(2750, 0)),
            new CalibrationTable.CalibrationValue<>(5.50, new ShooterSetpoint(3400, 0)),
            new CalibrationTable.CalibrationValue<>(6.50, new ShooterSetpoint(3700, 0)),
            new CalibrationTable.CalibrationValue<>(7.50, new ShooterSetpoint(4150, 0)),
            new CalibrationTable.CalibrationValue<>(8.50, new ShooterSetpoint(5000, 0)),
            new CalibrationTable.CalibrationValue<>(9.00, new ShooterSetpoint(5500, 0)),
            new CalibrationTable.CalibrationValue<>(Double.POSITIVE_INFINITY, new ShooterSetpoint(5500, 0)),
        };
    public static final CalibrationTable<ShooterSetpoint> passingTable = new CalibrationTable<>(passingValues);

    public static final InterpolatingDoubleTreeMap distanceToToF = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToPassingToF = new InterpolatingDoubleTreeMap();

	static {
		distanceToToF.put(1.681, 0.884);
		distanceToToF.put(2.485, (18.245-10.341)/8.0);
		distanceToToF.put(4.00, (17.879-8.171)/8.0);
		distanceToToF.put(5.20, (22.215-10.573)/8.0);
		distanceToToF.put(Double.POSITIVE_INFINITY, 2.0);

		distanceToPassingToF.put(4.95, ((22.419-10.541)/8.0 + (26.187-14.111)/8.0)/2.0);
		distanceToPassingToF.put(6.85, ((28.490-14.146)/8.0 + (33.829-19.382)/8.0)/2.0);
		distanceToPassingToF.put(8.49, ((21.951-6.839)/8.0 + (27.324-12.576)/8.0)/2.0);
		distanceToPassingToF.put(9.95, (29.525-12.610)/8.0);
		distanceToPassingToF.put(Double.POSITIVE_INFINITY, 2.2);
	}

	public static MotorConfig getFlywheelMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Flywheel A";
		config.port = Ports.FLYWHEEL_A;

		// Unloaded config -- values from SysId
        config.talonConfig.Slot0.kP = 0.15; // SysId == 0.075665
        config.talonConfig.Slot0.kI = 0.0;
        config.talonConfig.Slot0.kD = 0.0;
        config.talonConfig.Slot0.kS = 0.17763;
        config.talonConfig.Slot0.kG = 0.0;
        config.talonConfig.Slot0.kV = 0.1281;
        config.talonConfig.Slot0.kA = 0.018656;

		// Loaded config
        config.talonConfig.Slot1 = Slot1Configs.from(SlotConfigs.from(config.talonConfig.Slot0));
        config.talonConfig.Slot1.kS += 0.3; // Extra baseline to help with load

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

		config.talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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