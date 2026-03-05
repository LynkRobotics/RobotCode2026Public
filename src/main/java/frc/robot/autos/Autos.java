package frc.robot.autos;

import static frc.robot.Options.optMirrorAuto;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Aiming;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.AimOnly;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.superstructure.Superstructure;

public class Autos extends SubsystemBase {
    public static final Autos instance = new Autos();

    private final SendableChooser<Command> autoChooser;
    private final HashMap<Command, String> startingPaths = new HashMap<>();
    private final HashMap<String, Pose2d> startingPoses = new HashMap<>();
    private Pose2d startingPose = null;

    public Autos() {
        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> stream.filter(auto -> !auto.getName().startsWith("Dummy")));
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);

        SmartDashboard.putData("auto/Assume Starting Pose", LoggedCommands.runOnce("Assume starting pose", () -> Pose.instance.setPose(startingPose)).ignoringDisable(true));
        
        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        Autos.autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        Autos.autoNamedCommand("Stop", Commands.runOnce(Swerve.instance::stopSwerve));
        
        if (Constants.fullDashboard) {
            SmartDashboard.putData("auto/Debug Drive", Commands.sequence(
                PathCommand("Debug Drive"),
                Swerve.instance.Stop()));
        }
    }

    public static void autoNamedCommand(String name, Command command) {
        NamedCommands.registerCommand(name, LoggedCommands.logWithName(name + " (auto)", command));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void addAutoCommand(SendableChooser<Command> chooser, Command command) {
        chooser.addOption(command.getName(), command);
    }

    Command DynamicPath(LinkedList<Pose2d> poses, PathConstraints constraints, GoalEndState goal) {
        return LoggedCommands.defer("Dynamic Path", () -> {
                Pose2d currentPose = Pose.instance.getPose();
                if (Robot.isRed()) {
                    // All paths are defined based on blue
                    currentPose = FlippingUtil.flipFieldPose(currentPose);
                }
                poses.addFirst(currentPose);
                List<Waypoint> points = PathPlannerPath.waypointsFromPoses(poses);
                PathPlannerPath path = new PathPlannerPath(points, constraints, null, goal);
                if (optMirrorAuto.get()) path = path.mirrorPath();
                return AutoBuilder.followPath(path);
            }, Set.of(Swerve.instance));
    }

    public Command aimOnly() {
        return new AimOnly(Swerve.instance, Aiming::aimAtHub);
    }

    public Command deliverHopper(boolean timeout) {
        return LoggedCommands.deadline("Deliver hopper",
                timeout ? LoggedCommands.waitSeconds("Wait for hopper delivery", AutoConstants.quickShootingTime) : Commands.idle(),
                Superstructure.instance.DeliverToHub(),
                aimOnly()
            );
    }

    public void buildAutos(SendableChooser<Command> chooser) {
        Command farNear = LoggedCommands.sequence("Far NZ then near",
            LoggedCommands.deadline("Intake through far NZ",
                PathCommand("RTrench through far NZ"),
                Intake.getInstance().RunIntake()),
            PathCommand("Far NZ back to Right Trench"),
            Swerve.instance.Stop(),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through near NZ",
                PathCommand("RTrench through near NZ"),
                Intake.getInstance().RunIntake()),
            PathCommand("Near NZ back to Right Trench"),
            Swerve.instance.Stop(),
            deliverHopper(true)
        );

        startingPaths.put(farNear, "RTrench through far NZ");
        addAutoCommand(chooser, farNear);
        
        Command nearFar = LoggedCommands.sequence("Near NZ then far",
            Commands.deadline(
                Commands.sequence(
                    LoggedCommands.deadline("Intake through near NZ",
                        PathCommand("RTrench through near NZ"),
                        Intake.getInstance().RunIntake()),
                    PathCommand("Near NZ back to Right Trench"),
                    Swerve.instance.Stop()),
                Shooter.getInstance().Idle()),
            deliverHopper(true),

            Commands.deadline(
                Commands.sequence(
                    LoggedCommands.deadline("Intake through far NZ",
                        PathCommand("RTrench through far NZ"),
                        Intake.getInstance().RunIntake()),
                    PathCommand("Far NZ back shortcut"),
                    Swerve.instance.Stop()),
                Shooter.getInstance().Idle()),
            deliverHopper(true),

            // TODO What should third pass be?
            Commands.deadline(
                Commands.sequence(
                    LoggedCommands.deadline("Intake through far NZ",
                        PathCommand("RTrench through far NZ"),
                        Intake.getInstance().RunIntake()),
                    PathCommand("Far NZ back shortcut"),
                    Swerve.instance.Stop()),
                Shooter.getInstance().Idle()),
            deliverHopper(true)
        );

        startingPaths.put(nearFar, "RTrench through near NZ");
        addAutoCommand(chooser, nearFar);

        Command sweep = LoggedCommands.sequence("Sweep Auto",
            LoggedCommands.deadline("Intake through first pass",
                LoggedCommands.sequence("First pass",
                    PathCommand("RTrench through NZ"),
                    PathCommand("NZ back to Right Trench"),
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.sequence("Second pass",
                    PathCommand("RT Sweep"),
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(false)
        );

        startingPaths.put(sweep, "RTrench through near NZ");
        addAutoCommand(chooser, sweep);

        Command doublePass = LoggedCommands.sequence("Double Pass",
            LoggedCommands.deadline("Intake through NZ end",
                PathCommand("RTrench pass through fuel"),
                Intake.getInstance().RunIntake()),
            PathCommand("RTrench return from fuel"),
            Swerve.instance.Stop(),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through NZ (2)",
                PathCommand("RTrench second pass through fuel"),
                Intake.getInstance().RunIntake()),
            PathCommand("RTrench second return from fuel"),
            Swerve.instance.Stop(),
            deliverHopper(true)
        );

        startingPaths.put(doublePass, "RTrench pass through fuel");
        addAutoCommand(chooser, doublePass);

        Command middleBump = LoggedCommands.sequence("Bump middle",
            Commands.deadline(
                Commands.sequence(
                    LoggedCommands.deadline("Intake over bump through middle",
                        PathCommand("Bump through middle"),
                        Intake.getInstance().RunIntake()),
                    PathCommand("Middle return over bump"),
                    Swerve.instance.Stop()),
                Shooter.getInstance().Idle()),
            deliverHopper(false));

        startingPaths.put(middleBump, "Bump through middle");
        addAutoCommand(chooser, middleBump);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public Command PathCommand(String pathName) {
        Command pathCommand, mirrorCommand;
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            PathPlannerPath mirror = path.mirrorPath();

            pathCommand = AutoBuilder.followPath(path);
            pathCommand.setName("Follow PathPlanner path \"" + pathName + "\"");
            startingPoses.put(pathName, new Pose2d(path.getPathPoses().get(0).getTranslation(), path.getIdealStartingState().rotation()));

            mirrorCommand = AutoBuilder.followPath(mirror);
            mirrorCommand.setName("Follow Mirrored PathPlanner path \"" + pathName + "\"");
            startingPoses.put(pathName + " - Mirror", new Pose2d(mirror.getPathPoses().get(0).getTranslation(), mirror.getIdealStartingState().rotation()));
        } catch (Exception exception) {
            LoggedAlert.Error("PathPlanner", "Failed to load path \"" + pathName + "\"", exception.getMessage());
            return LoggedCommands.log("Missing PathPlanner path due to failure to load \"" + pathName + "\": " + exception.getMessage());
        }

        return LoggedCommands.either("Choosing auto path for " + pathName,
            LoggedCommands.logWithName("Mirrored Path: " + pathName, mirrorCommand),
            LoggedCommands.logWithName("Path: " + pathName, pathCommand),
            optMirrorAuto::get);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            return;
        }

        Command autoCommand = getAutonomousCommand();
        String poseDifference = "N/A";
        boolean differenceOK = false;

        if (autoCommand != null) {
            String firstPath = startingPaths.get(autoCommand);

            if (firstPath != null) {
                startingPose = startingPoses.get(firstPath + (optMirrorAuto.get() ? " - Mirror" : ""));
                DogLog.log("Autos/Starting Pose", startingPose);

                if (startingPose != null) {
                    Pose2d currentPose = Pose.instance.getPose();
                   
                    poseDifference = String.format("(%1.1f, %1.1f) @ %1.0f deg",
                        Units.Meters.of(currentPose.getX() - startingPose.getX()).in(Units.Inches),
                        Units.Meters.of(currentPose.getY() - startingPose.getY()).in(Units.Inches),
                        startingPose.getRotation().minus(currentPose.getRotation()).getDegrees());

                    if (Math.abs(currentPose.getX() - startingPose.getX()) < AutoConstants.maxSetupXError &&
                        Math.abs(currentPose.getY() - startingPose.getY()) < AutoConstants.maxSetupYError &&
                        Math.abs(startingPose.getRotation().minus(currentPose.getRotation()).getDegrees()) < AutoConstants.maxSetupDegError) {
                        differenceOK = true;
                    }
                }
            }
        }

        SmartDashboard.putString("autoSetup/Starting Pose Error", poseDifference);
        SmartDashboard.putBoolean("autoSetup/Starting Pose OK", differenceOK);
        SmartDashboard.putBoolean("autoSetup/Red Alliance?", Robot.isRed());
    }
}