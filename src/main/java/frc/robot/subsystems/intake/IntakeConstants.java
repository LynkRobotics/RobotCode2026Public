package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.LynkMotor.MotorConfig;
import frc.robot.Ports;

public class IntakeConstants {
    public static final double gearing = (18.0 / 24.0);
	public static final Voltage intakeVoltage = Units.Volts.of(12.0);
	public static final Voltage intakeReverseVoltage = Units.Volts.of(-8.0);
	public static final double pulseOutTime = 0.20;
	public static final double pulseInTime = 0.30;
	public static final double intakeCurrentDiffWarningLimit = 15.0;
	public static final double intakeCurrentDiffWarningTime = 0.50;
	public static final double intakeCurrentFixTime = 0.50;

	public enum IntakeMode {
		IDLE,
		PULSING,
		REVERSE,
		ACTIVE
	}

	public static MotorConfig getMotorConfig() {
		MotorConfig config = new MotorConfig();

		config.name = "Intake A";
		config.port = Ports.INTAKE_A;

        // config.talonConfig.Slot0.kP = 300.0;
		// config.talonConfig.Slot0.kD = 0.0;
		// config.talonConfig.Slot0.kS = 0.0;
		// config.talonConfig.Slot0.kG = 0.0;

		// config.talonConfig.MotionMagic.MotionMagicAcceleration = 1000;
		// config.talonConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
		// config.talonConfig.MotionMagic.MotionMagicJerk = 100;

		config.talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		config.talonConfig.CurrentLimits.StatorCurrentLimit = 60.0;

		config.talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.talonConfig.CurrentLimits.SupplyCurrentLimit = 80.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.talonConfig.CurrentLimits.SupplyCurrentLowerTime = 0.30;

		config.talonConfig.Voltage.PeakForwardVoltage = 12.0;
		config.talonConfig.Voltage.PeakReverseVoltage = -12.0;

		config.talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		config.talonConfig.Feedback.SensorToMechanismRatio = gearing;
		config.talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        return config;
    }
}