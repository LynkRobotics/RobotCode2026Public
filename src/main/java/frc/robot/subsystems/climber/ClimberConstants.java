package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.util.LynkMotor.MotorConfig;
import frc.robot.Ports;

public class ClimberConstants {
    public static final double gearing = (1.0 / 1.0);

    	public static MotorConfig getMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Climber";
		config.port = Ports.CLIMBER;

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
		config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

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