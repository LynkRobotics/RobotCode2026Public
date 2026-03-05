package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.util.LynkMotor.MotorConfig;
import frc.robot.Ports;

public class FeederConstants {
    public static final double feederGearing = (24.0 / 18.0);
    public static final double floorGearing = (18.0 / 18.0);

	public static final double feederSpeed = 1.0;
	public static final double floorSpeed = 1.0;

	public static MotorConfig getFeederMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Feeder";
		config.port = Ports.FEEDER;

        // config.talonConfig.Slot0.kP = 300.0;
		// config.talonConfig.Slot0.kD = 0.0;
		// config.talonConfig.Slot0.kS = 0.0;
		// config.talonConfig.Slot0.kG = 0.0;
		// config.talonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		// config.talonConfig.MotionMagic.MotionMagicAcceleration = 1000;
		// config.talonConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
		// config.talonConfig.MotionMagic.MotionMagicJerk = 100;

		// config.talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// config.talonConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
		// config.talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		// config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1; 

		config.talonConfig.Voltage.PeakForwardVoltage = 12.0;
		config.talonConfig.Voltage.PeakReverseVoltage = -12.0;

		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 	ClimberPosition.STOWED.angle.plus(maxExtension).in(Units.Rotations);

		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 	ClimberPosition.FULLY_STOWED.angle.in(Units.Rotations);

		config.talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		config.talonConfig.Feedback.SensorToMechanismRatio = feederGearing;
		config.talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // TODO Consider Brake to abort feeding

        return config;
    }

	public static MotorConfig getFloorMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Floor";
		config.port = Ports.FLOOR;

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
		config.talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.40;

		config.talonConfig.Voltage.PeakForwardVoltage = 12.0;
		config.talonConfig.Voltage.PeakReverseVoltage = -12.0;

		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
		// 	ClimberPosition.STOWED.angle.plus(maxExtension).in(Units.Rotations);

		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		// config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
		// 	ClimberPosition.FULLY_STOWED.angle.in(Units.Rotations);

		config.talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.talonConfig.Feedback.SensorToMechanismRatio = floorGearing;
		config.talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return config;
    }
}