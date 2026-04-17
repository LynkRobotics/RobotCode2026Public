package frc.robot.autos;

import static frc.robot.Options.optMirrorAuto;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.autos.AutoConstants.AutoBuilderConfig;
import frc.robot.autos.AutoConstants.AutoBuilderConfig.FirstPriority;
import frc.robot.autos.AutoConstants.AutoBuilderConfig.FuelIntakeDepth;
import frc.robot.autos.AutoConstants.AutoBuilderConfig.IntakeSpeed;
import frc.robot.autos.AutoConstants.AutoBuilderConfig.StartingPoints;
import frc.robot.commands.AimOnly;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.superstructure.Superstructure;

public class Autos extends SubsystemBase {
    public static final Autos instance = new Autos();

    private final SendableChooser<Command> autoChooser;
    private final HashMap<Command, String> startingPaths = new HashMap<>();
    private final HashMap<String, Pose2d> startingPoses = new HashMap<>();
    private Pose2d startingPose = null;
    private AutoBuilderConfig currentAutoBuilderConfig = null;
    private Command autoBuilderCommand = null;
    private Command autoBuilderIndirect = Commands.defer(() -> autoBuilderCommand == null ? Commands.idle() : autoBuilderCommand, Set.of());
    private HashMap<String, SendableChooser<? extends Enum<?>>> autoBuilderChoosers = new HashMap<>();

    public Autos() {
        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> stream.filter(auto -> !auto.getName().startsWith("Demo")));
        SmartDashboard.putData("auto/Auto Chooser", autoChooser);
        buildAutos(autoChooser);
        autoChooser.addOption("-- Auto Builder --", autoBuilderIndirect);

        SmartDashboard.putData("auto/Assume Starting Pose", LoggedCommands.runOnce("Assume starting pose", () -> Pose.instance.setPose(startingPose == null ? Pose2d.kZero : startingPose)).ignoringDisable(true));
        
        // Default named commands for PathPlanner
        SmartDashboard.putNumber("auto/Startup delay", 0.0);
        Autos.autoNamedCommand("Startup delay", Commands.defer(() -> Commands.waitSeconds(SmartDashboard.getNumber("auto/Startup delay", 0.0)), Set.of()));
        Autos.autoNamedCommand("Stop", Commands.runOnce(Swerve.instance::stopSwerve));
        
        if (Constants.fullDashboard) {
            SmartDashboard.putData("auto/Debug Drive", Commands.sequence(
                PathCommand("Debug Drive"),
                Swerve.instance.Stop()));
        }

        createAutoOptions();
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
                Commands.sequence(
                    Vision.instance.ForceVisionUpdate().withTimeout(AutoConstants.visionWaitTime),
                    LoggedCommands.waitUntil("Wait for alignment", Aiming::isHubAligned),
                    Superstructure.instance.DeliverToHub()),
                aimOnly()
            );
    }

    private PathPlannerPath shortenFuelPath(PathPlannerPath original) {
        List<Waypoint> waypoints = original.getWaypoints();
        Waypoint lastWaypoint = waypoints.get(waypoints.size() - 1);
        Translation2d position = lastWaypoint.anchor().minus(AutoConstants.pathShortening);
        Waypoint shortWaypoint = new Waypoint(lastWaypoint.prevControl(), position, lastWaypoint.nextControl());
        waypoints.set(waypoints.size() - 1, shortWaypoint);
        return new PathPlannerPath(
            waypoints,
            original.getRotationTargets(),
            original.getPointTowardsZones(),
            original.getConstraintZones(),
            original.getEventMarkers(),
            original.getGlobalConstraints(),
            original.getIdealStartingState(),
            original.getGoalEndState(),
            false);
    }

    private PathPlannerPath shortenStartPosition(PathPlannerPath original) {
        List<Waypoint> waypoints = original.getWaypoints();
        Waypoint firstWaypoint = waypoints.get(0);
        Translation2d position = firstWaypoint.anchor().minus(AutoConstants.pathShortening);
        Waypoint shortWaypoint = new Waypoint(firstWaypoint.prevControl(), position, firstWaypoint.nextControl());
        waypoints.set(0, shortWaypoint);
        return new PathPlannerPath(
            waypoints,
            original.getRotationTargets(),
            original.getPointTowardsZones(),
            original.getConstraintZones(),
            original.getEventMarkers(),
            original.getGlobalConstraints(),
            original.getIdealStartingState(),
            original.getGoalEndState(),
            false);
    }

    private PathPlannerPath slowFuelIntake(PathPlannerPath original) {
        List<ConstraintsZone> constraintZones = original.getConstraintZones();
        ConstraintsZone lastZone = constraintZones.get(constraintZones.size() - 1);
        PathConstraints constraints = lastZone.constraints();
        PathConstraints slowConstraints = new PathConstraints(AutoConstants.slowIntakeMaxVel, constraints.maxAcceleration(), constraints.maxAngularVelocity(), constraints.maxAngularAcceleration());
        ConstraintsZone slowZone = new ConstraintsZone(lastZone.minPosition(), lastZone.maxPosition(), slowConstraints);
        constraintZones.set(constraintZones.size() - 1, slowZone);
        return new PathPlannerPath(
            original.getWaypoints(),
            original.getRotationTargets(),
            original.getPointTowardsZones(),
            constraintZones,
            original.getEventMarkers(),
            original.getGlobalConstraints(),
            original.getIdealStartingState(),
            original.getGoalEndState(),
            false);
    }

    private PathPlannerPath slowSweep(PathPlannerPath original) {
        List<ConstraintsZone> constraintZones = original.getConstraintZones();
        ConstraintsZone lastZone = constraintZones.get(0);
        PathConstraints constraints = lastZone.constraints();
        PathConstraints slowConstraints = new PathConstraints(AutoConstants.slowSweepMaxVel, constraints.maxAcceleration(), constraints.maxAngularVelocity(), constraints.maxAngularAcceleration());
        ConstraintsZone slowZone = new ConstraintsZone(lastZone.minPosition(), lastZone.maxPosition(), slowConstraints);
        constraintZones.set(0, slowZone);
        return new PathPlannerPath(
            original.getWaypoints(),
            original.getRotationTargets(),
            original.getPointTowardsZones(),
            constraintZones,
            original.getEventMarkers(),
            original.getGlobalConstraints(),
            original.getIdealStartingState(),
            original.getGoalEndState(),
            false);
    }

    private String capitalize(String str) {
        return str.substring(0, 1).toUpperCase() + str.substring(1).toLowerCase();
    }

    public Command Recover() {
        return LoggedCommands.sequence("Recovery Auto", 
            LoggedCommands.deadline("Safe recovery over bump",
                Commands.sequence(
                    // TODO Drive to recovery position based on vision
                    PathCommand("Traverse Bump")),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Swerve.instance.Stop(),
            deliverHopper(true),
            PathCommand("Bump to Trench")
        );
    }

    public Command buildAuto(AutoBuilderConfig config) {
        String autoName = "Auto";
        String firstPathName = "";
        Command firstPath;
        Function<PathPlannerPath,PathPlannerPath> pathModifier = (p) -> p;
        String returnPathName = "";
        Command returnPath;
        String sweepPathName = "";
        Command sweepPath;

        firstPathName = capitalize(config.startingPoint().toString());
        if (config.firstPriority() == FirstPriority.DISRUPT) {
            firstPathName = "Disrupt";
            firstPath = PathCommand(firstPathName);
            returnPathName = "[N/A]";
            returnPath = Commands.none();
        } else if (config.startingPoint() == StartingPoints.TRENCH) {
            firstPathName += " - " + capitalize(config.firstPriority().toString());
            firstPathName += " - " + capitalize(config.firstPassDepth().toString());
            if (config.fuelIntakeDepth() == FuelIntakeDepth.SHORT) {
                autoName += " (Short)";
                pathModifier = pathModifier.andThen(this::shortenFuelPath);
            }
            if (config.intakeSpeed() == IntakeSpeed.SLOW) {
                autoName += " (Slow)";
                pathModifier = pathModifier.andThen(this::slowFuelIntake);
            }
            firstPath = PathCommand(firstPathName, pathModifier);
            returnPathName = capitalize(config.returnMethod().toString()) + " Return";
            returnPathName += " - " + capitalize(config.firstPassDepth().toString());
            returnPath = PathCommand(returnPathName, (config.fuelIntakeDepth() == FuelIntakeDepth.SHORT) ? this::shortenStartPosition : null);
        } else {
            firstPath = PathCommand(firstPathName);
            returnPathName = "Bump Return";
            returnPath = PathCommand(returnPathName);
        }

        sweepPathName = "Sweep - " + capitalize(config.sweepType().toString());
        sweepPath = PathCommand(sweepPathName, (config.sweepSpeed() == IntakeSpeed.SLOW) ? this::slowSweep : null);

        autoName += ": " + firstPathName + " -- " + returnPathName + " -- " + sweepPathName;
        Command autoCommand = LoggedCommands.sequence(autoName,
            LoggedCommands.deadline("Intake through first pass",
                // TODO Add an abort option to return over bump if interfered with
                LoggedCommands.sequence("First pass",
                    LoggedCommands.race("Stay level through first path",
                        firstPath,
                        Pose.instance.StayLevel()),
                    returnPath,
                    Swerve.instance.Stop()),
                LoggedCommands.sequence("Deploy and run Intake",
                    Feeder.getInstance().DeployIntake(),
                    Intake.getInstance().RunIntake()),
                Shooter.getInstance().Idle()),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.sequence("Second pass",
                    sweepPath,
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(false)
        );

        startingPaths.put(autoBuilderIndirect, firstPathName);

        return autoCommand;
    }


    public void buildAutos(SendableChooser<Command> chooser) {
        Command sweep = LoggedCommands.sequence("Default - Double Bump Auto",
            LoggedCommands.deadline("Intake through first pass",
                LoggedCommands.sequence("First pass",
                    PathCommand("Trench - Midline - Middle"),
                    PathCommand("Bump Return - Middle"),
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.sequence("Second pass",
                    PathCommand("Sweep - Bump"),
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(false)
        );

        startingPaths.put(sweep, "Trench - Midline - Middle");
        addAutoCommand(chooser, sweep);

        Command commonAuto = LoggedCommands.sequence("Common Auto",
            LoggedCommands.deadline("Intake through first pass",
                LoggedCommands.sequence("First pass",
                    LoggedCommands.race("Stay level through first path",
                        PathCommand("Trench - Midline - Far - Slow"),
                        Pose.instance.StayLevel()),
                    PathCommand("Bump Return - Middle"),
                    Swerve.instance.Stop()),
                LoggedCommands.sequence("Deploy and run Intake",
                    Feeder.getInstance().DeployIntake(),
                    Intake.getInstance().RunIntake()),
                Shooter.getInstance().Idle()),
            deliverHopper(true),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.sequence("Second pass",
                    PathCommand("Sweep - Bump"),
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(false)
        );

        startingPaths.put(commonAuto, "Trench - Midline - Far - Slow");
        addAutoCommand(chooser, commonAuto);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public Command PathCommand(String pathName) {
        return PathCommand(pathName, null);
    }

    public Command PathCommand(String pathName, Function<PathPlannerPath, PathPlannerPath> pathModifier) {
        Command pathCommand, mirrorCommand;
        
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            if (pathModifier != null) {
                path = pathModifier.apply(path);
            }
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

    private <E extends Enum<E>> void createAutoOption(Class<E> enumClass, String optName) {
        SendableChooser<E> chooser = new SendableChooser<>();
        boolean defaultSet = false;

        EnumSet<E> options = EnumSet.allOf(enumClass);
        for (E option : options) {
            if (!defaultSet) {
                chooser.setDefaultOption(option.toString(), option);
                defaultSet = true;
            } else {
                chooser.addOption(option.toString(), option);
            }
        }

        SmartDashboard.putData("auto/AutoBuilderOption/" + optName, chooser);
        autoBuilderChoosers.put(optName, chooser);
    }

    private void createAutoOptions() {
        createAutoOption(AutoBuilderConfig.StartingPoints.class, "Starting Point");
        createAutoOption(AutoBuilderConfig.FirstPriority.class, "First Priority");
        createAutoOption(AutoBuilderConfig.PassDepth.class, "First Pass Depth");
        createAutoOption(AutoBuilderConfig.FuelIntakeDepth.class, "Fuel Intake Depth");
        createAutoOption(AutoBuilderConfig.IntakeSpeed.class, "Intake Speed");
        createAutoOption(AutoBuilderConfig.CrossingPoints.class, "Return Method");
        createAutoOption(AutoBuilderConfig.SweepType.class, "Sweep Type");
        createAutoOption(AutoBuilderConfig.IntakeSpeed.class, "Sweep Speed");
    }

    @SuppressWarnings("unchecked")
    private AutoBuilderConfig getAutoBuilderConfig() {
        return new AutoBuilderConfig(
            ((SendableChooser<AutoBuilderConfig.StartingPoints>)autoBuilderChoosers.get("Starting Point")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.FirstPriority>)autoBuilderChoosers.get("First Priority")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.PassDepth>)autoBuilderChoosers.get("First Pass Depth")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.FuelIntakeDepth>)autoBuilderChoosers.get("Fuel Intake Depth")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.IntakeSpeed>)autoBuilderChoosers.get("Intake Speed")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.CrossingPoints>)autoBuilderChoosers.get("Return Method")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.SweepType>)autoBuilderChoosers.get("Sweep Type")).getSelected(),
            ((SendableChooser<AutoBuilderConfig.IntakeSpeed>)autoBuilderChoosers.get("Sweep Speed")).getSelected());
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            return;
        }

        AutoBuilderConfig newAutoBuilderConfig = getAutoBuilderConfig();
        if (!newAutoBuilderConfig.equals(currentAutoBuilderConfig)) {
            currentAutoBuilderConfig = newAutoBuilderConfig;
            autoBuilderCommand = buildAuto(currentAutoBuilderConfig);
        }

        Command autoCommand = getAutonomousCommand();
        String poseDifference = "N/A";
        boolean differenceOK = false;

        if (autoCommand != null) {
            String firstPath = startingPaths.get(autoCommand);

            if (firstPath != null) {
                startingPose = Pose.flipIfRed(startingPoses.get(firstPath + (optMirrorAuto.get() ? " - Mirror" : "")));
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