package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Ports;

public class SwerveConstants {
    // Multiplier for "slow mode" driving
    public static final double slowMode = 0.5;
    
    public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK5n.KrakenX60X44(COTSTalonFXSwerveConstants.SDS.MK5n.driveRatios.R2);

    /* Drivetrain Constants */
    public static final Distance trackWidth = Units.Inches.of(20.75);
    /* Center to Center distance of left and right modules in meters. */
    public static final Distance wheelBase = Units.Inches.of(20.75);
    /* Center to Center distance of front and rear module wheels in meters. */
    public static final double wheelCircumference = chosenModule.wheelCircumference * 0.98685; // based on testing

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase.div(2.0), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(2.0), trackWidth.div(2.0).unaryMinus()),
        new Translation2d(wheelBase.div(2.0).unaryMinus(), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(2.0).unaryMinus(), trackWidth.div(2.0).unaryMinus()));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 40;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveStatorCurrentLimit = 80; 
    public static final int driveSupplyCurrentLimit = 60;
    public static final int driveCurrentThreshold = 60; 
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 1.1315; // NOTE: This must be tuned to specific robot
    // 0.89558 for Default
    /* After completeing characterization and inserting 
     * the KS, KV, and KA values into the code, tune the 
     * drive motor kP until it doesn't overshoot and 
     * doesnt oscilate around a target velocity. */
    public static final double driveKI = 0.0; //Leave driveKI at 0.0
    public static final double driveKD = 0.0; //Leave driveKD at 0.0
    public static final double driveKF = 0.0; //Leave driveKF at 0.0 

    /* Drive Motor Characterization Values From SYSID */ 
    public static final double driveKS = 0.28416;
    public static final double driveKV = 0.73994;
    public static final double driveKA = 0.072982;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5.12; // MK5n R2
    /** Radians per Second */
    public static final double driveRadius = Math.hypot(wheelBase.in(Units.Meters), trackWidth.in(Units.Meters)) / 2.0;
    public static final double maxAngularVelocity = maxSpeed / driveRadius;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    public static final double maxAngleError = 2.0; // Degrees before we alert that the module is not aligned

    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = 0;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 0;
        public static final CANBus canBus = Ports.Bus.SWERVE.bus;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-111.4);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBus, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { 
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 1;
        public static final CANBus canBus = Ports.Bus.SWERVE.bus;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(21.7);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBus, angleOffset);
    }
        
    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 2;
        public static final CANBus canBus = Ports.Bus.SWERVE.bus;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.4);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBus, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 3;
        public static final CANBus canBus = Ports.Bus.SWERVE.bus;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(171.3);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBus, angleOffset);
    }
}