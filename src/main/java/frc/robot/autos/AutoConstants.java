package frc.robot.autos;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

public final class AutoConstants {
    public static final double maxSetupXError = Units.Inches.of(4.0).in(Units.Meters);
    public static final double maxSetupYError = Units.Inches.of(8.0).in(Units.Meters);
    public static final double maxSetupDegError = 15.0;
    public static final double quickShootingTime = Constants.hopperShootingTime - 0.40;
    public static final Translation2d pathShortening = new Translation2d(Units.Meters.of(0.0), Units.Meters.of(1.1));
    public static final LinearVelocity slowIntakeMaxVel = Units.MetersPerSecond.of(1.1);
    public static final LinearVelocity slowSweepMaxVel = Units.MetersPerSecond.of(1.5);
    public static final double visionWaitTime = 0.5;

    public static record AutoBuilderConfig(
        StartingPoints startingPoint,
        FirstPriority firstPriority,
        PassDepth firstPassDepth,
        FuelIntakeDepth fuelIntakeDepth,
        IntakeSpeed intakeSpeed,
        CrossingPoints returnMethod,
        SweepType sweepType,
        IntakeSpeed sweepSpeed)
    {
        public enum StartingPoints { TRENCH, BUMP };
        public enum FirstPriority { MIDLINE, FUEL, DISRUPT };
        public enum PassDepth { FAR, MIDDLE, NEAR };
        public enum FuelIntakeDepth { FULL, SHORT };
        public enum IntakeSpeed { DEFAULT, SLOW };
        public enum CrossingPoints { BUMP, TRENCH };
        public enum SweepType { BUMP, TRENCH, NARROW };
    }
    
    // TODO Find out why this doesn't work
    public static final RobotConfig robotConfig = new RobotConfig(
        Mass.ofRelativeUnits(150, Pounds),
        MomentOfInertia.ofRelativeUnits(5.53, KilogramSquareMeters),
        new ModuleConfig(
            SwerveConstants.wheelCircumference / (Math.PI * 2.0),
            SwerveConstants.maxSpeed, 
            1.5,
            DCMotor.getKrakenX60Foc(1),
            SwerveConstants.chosenModule.driveGearRatio,
            SwerveConstants.driveStatorCurrentLimit,
            1),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0), SwerveConstants.trackWidth.div(2.0).unaryMinus()),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)),
        new Translation2d(SwerveConstants.wheelBase.div(2.0).unaryMinus(), SwerveConstants.trackWidth.div(2.0)).unaryMinus());
}