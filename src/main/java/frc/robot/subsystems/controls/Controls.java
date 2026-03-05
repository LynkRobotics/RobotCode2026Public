package frc.robot.subsystems.controls;

import static frc.robot.Options.optAutoAiming;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Aiming;
import frc.robot.Constants;
import frc.robot.Field.PassTarget;
import frc.robot.autos.Autos;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.commands.TeleopSwerve;

public class Controls extends SubsystemBase {
    public static final Controls instance = new Controls();

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translation = driver::getLeftY;
    private final Supplier<Double> strafe = driver::getLeftX;
    private final Supplier<Double> rotation = driver::getRightX;

    public Controls() {
        SmartDashboard.putNumber("TeleOp Speed Governor", 1.0);
        // SmartDashboard.putNumber("TeleOp Translation Expo", 2.0);
        // SmartDashboard.putNumber("TeleOp Rotation Expo", 2.0);

        // TODO Restore
        // Pose pose = Pose.instance;
        // SmartDashboard.putData(LoggedCommands.runOnce("Zero Gyro", pose::zeroGyro));
        // SmartDashboard.putData(LoggedCommands.runOnce("Reset heading", pose::resetHeading));

        if (Constants.fullDashboard) {
            Swerve swerve = Swerve.instance;
            SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Coast", swerve::setMotorsToCoast, swerve).ignoringDisable(true));
            SmartDashboard.putData(LoggedCommands.runOnce("autoSetup/Set Swerve Brake", swerve::setMotorsToBrake, swerve).ignoringDisable(true));
            SmartDashboard.putData(LoggedCommands.run("autoSetup/Set Swerve Aligned", swerve::alignStraight, swerve).ignoringDisable(true));
        }
    }

    public Command Rumble() {
        return Commands.deadline(
            Commands.waitSeconds(0.5),
            LoggedCommands.startEnd("Rumble",
                () -> {
                    driver.setRumble(RumbleType.kLeftRumble, 1.0);
                    driver.setRumble(RumbleType.kRightRumble, 1.0);
                },
                () -> {
                    driver.setRumble(RumbleType.kLeftRumble, 0.0);
                    driver.setRumble(RumbleType.kRightRumble, 0.0);
                }));
    }

    public Command TriggerRumble() {
        Command rumbleCommmand = Rumble();

        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(rumbleCommmand));
    }

    public void configureButtonBindings() {
        boolean sysIdSteer = false;
        boolean sysIdDrive = false;
        boolean testControls = false;

        if (sysIdSteer) {
            driver.a().whileTrue(Swerve.instance.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward).finallyDo(Swerve.instance::stopSwerve));
            driver.b().whileTrue(Swerve.instance.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse).finallyDo(Swerve.instance::stopSwerve));
            driver.x().whileTrue(Swerve.instance.sysIdSteerDynamic(SysIdRoutine.Direction.kForward).finallyDo(Swerve.instance::stopSwerve));
            driver.y().whileTrue(Swerve.instance.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse).finallyDo(Swerve.instance::stopSwerve));
            driver.leftBumper().onTrue(LoggedCommands.runOnce("Start signal logger", SignalLogger::start));
            driver.rightBumper().onTrue(LoggedCommands.runOnce("Stop signal logger", SignalLogger::stop));
        } else if (sysIdDrive) {
            driver.a().whileTrue(Swerve.instance.sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward).finallyDo(Swerve.instance::stopSwerve));
            driver.b().whileTrue(Swerve.instance.sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse).finallyDo(Swerve.instance::stopSwerve));
            driver.x().whileTrue(Swerve.instance.sysIdDriveDynamic(SysIdRoutine.Direction.kForward).finallyDo(Swerve.instance::stopSwerve));
            driver.y().whileTrue(Swerve.instance.sysIdDriveDynamic(SysIdRoutine.Direction.kReverse).finallyDo(Swerve.instance::stopSwerve));
            driver.leftBumper().onTrue(LoggedCommands.runOnce("Start signal logger", SignalLogger::start));
            driver.rightBumper().onTrue(LoggedCommands.runOnce("Stop signal logger", SignalLogger::stop));
        } else if (testControls) {
            driver.a().whileTrue(Superstructure.instance.DeliverToHub());
            driver.x().whileTrue(Shooter.getInstance().Shoot().finallyDo(Shooter.getInstance()::stop));
            driver.y().whileTrue(Feeder.getInstance().RunFeederOnly());
            driver.b().whileTrue(Feeder.getInstance().RunFloorOnly());
            driver.leftBumper().whileTrue(Intake.getInstance().RunIntake());
            driver.rightBumper().whileTrue(Intake.getInstance().Pulse());
        } else {
            driver.rightTrigger().whileTrue(Superstructure.instance.SmartShoot());
            driver.rightBumper().whileTrue(Superstructure.instance.DeliverToHub());
            driver.leftBumper().whileTrue(Superstructure.instance.Intake());
            driver.leftTrigger().whileTrue(Superstructure.instance.Pass());
            driver.a().onTrue(LoggedCommands.runOnce("Set Pass CORNER", () -> Aiming.setPassTarget(PassTarget.CORNER)));
            driver.x().onTrue(LoggedCommands.runOnce("Set Pass MIDDLE", () -> Aiming.setPassTarget(PassTarget.MIDDLE)));
            driver.b().onTrue(LoggedCommands.runOnce("Set Pass TRENCH", () -> Aiming.setPassTarget(PassTarget.TRENCH)));
        }
        driver.rightStick().onTrue(LoggedCommands.runOnce("Toggle automatic aiming", optAutoAiming::toggle));
        driver.back().whileTrue(Superstructure.instance.Unjam());

        if (Constants.atHQ) {
            // driver.povUp().whileTrue(
            //     Commands.sequence(
            //         new PIDSwerve(Swerve.instance, Pose.instance, new Pose2d(0.0, 0.0, Rotation2d.kZero), false, true),
            //         // new PIDSwerve(Swerve.instance, Pose.instance, new Pose2d(4.52, 1.71, Rotation2d.fromDegrees(-90.4)), false, true),
            //         Swerve.instance.Stop(),
            //         Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached Debug Position"))
            //         // Commands.runOnce(() -> LEDSubsystem.triggerError())
            //         ));
            driver.povRight().onTrue(
                LoggedCommands.sequence("Test Drive -- 5 meters",
                    Commands.runOnce(() -> Pose.instance.setPose(new Pose2d(2.0, 7.0, Rotation2d.kCW_90deg))),
                    Autos.instance.PathCommand("Test Drive - 5m"),
                    Swerve.instance.Stop(),
                    Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached End of Path")),
                    Commands.runOnce(() -> LED.triggerError())));
            driver.povLeft().onTrue(
                LoggedCommands.sequence("Test Drive -- 2.5 meters",
                    Commands.runOnce(() -> Pose.instance.setPose(Pose.flipIfRed(new Pose2d(2.0, 1.0, Rotation2d.kZero)))),
                    LoggedCommands.logWithName("2.5 m path", Autos.instance.PathCommand("Test Drive - 2.5m")),
                    LoggedCommands.logWithName("Stop", Swerve.instance.Stop()),
                    Commands.runOnce(() -> LoggedAlert.Info("Debug", "In Position", "Reached End of Path")),
                    LoggedCommands.runOnce("Test End", () -> LED.triggerError())
                ));

            driver.povUp().onTrue(LoggedCommands.runOnce("Drive forward test", () -> Swerve.instance.drive(new Translation2d(0.0, 1.0), 0.0, false), Swerve.instance).finallyDo(Swerve.instance::stopSwerve));
            driver.povDown().onTrue(LoggedCommands.runOnce("Drive backward test", () -> Swerve.instance.drive(new Translation2d(0.0, -1.0), 0.0, false), Swerve.instance).finallyDo(Swerve.instance::stopSwerve));

            // DogLog.log("Debug/Target pose", Pose.flipIfRed(new Pose2d(2.0, 1.0, Rotation2d.kZero)));
            // DogLog.log("Debug/Target pose", Pose.flipIfRed(new Pose2d(4.20, 0.450, Rotation2d.kCCW_90deg)));
        }
    }

    private double speedLimitFactor() {
        return 1.0;
    }

    public Command TeleOpSwerve() {
        TeleopSwerve command = new TeleopSwerve(
                Swerve.instance,
                () -> -translation.get() * Constants.driveStickSensitivity,
                () -> -strafe.get() * Constants.driveStickSensitivity,
                () -> -rotation.get() * Constants.turnStickSensitivity,
                this::speedLimitFactor
            );
        command.setAutoAimSupplier(Aiming::autoAim);
        return command;
    }
}