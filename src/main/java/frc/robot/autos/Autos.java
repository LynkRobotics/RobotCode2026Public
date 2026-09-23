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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.commands.pidswerve.PIDSwerve;
import frc.robot.commands.pidswerve.PIDSwerveConstants.PIDSpeed;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveAlign;
import frc.robot.subsystems.swerve.SwerveConstants;
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
    private boolean monitorTilt = true;

    public Autos() {
        // Build an autoChooser (defaults to none)
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> stream.filter(auto -> !auto.getName().startsWith("Demo")));
        SmartDashboard.putData("Autos/Auto Chooser", autoChooser);
        buildAutos(autoChooser);
        // autoChooser.addOption("-- Auto Builder --", autoBuilderIndirect);

        new EventTrigger("Before Bump").onTrue(IgnoreTilt());

        SmartDashboard.putData("Autos/Assume Starting Pose", LoggedCommands.runOnce("Assume starting pose", () -> Pose.instance.setPose(startingPose == null ? Pose2d.kZero : startingPose)).ignoringDisable(true));
                
        if (Constants.fullDashboard) {
            SmartDashboard.putData("Autos/Debug Drive", Commands.sequence(
                PathCommand("Debug Drive"),
                Swerve.instance.Stop()));
        }

        createAutoOptions();
    }

    private Translation2d mirrorTranslation(Translation2d position) {
        return new Translation2d(position.getX(), FlippingUtil.fieldSizeY - position.getY());
    }

    private Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(mirrorTranslation(pose.getTranslation()), pose.getRotation().unaryMinus());
    }

    public Command MonitorTilt() {
        return LoggedCommands.runOnce("Monitor tilt", () -> {
            Pose.instance.resetTilt();
            monitorTilt = true;
            Pose.instance.setCountTilts(monitorTilt);
        });
    }

    public Command IgnoreTilt() {
        return LoggedCommands.runOnce("Ignore tilt", () -> {
            monitorTilt = false;
            Pose.instance.setCountTilts(monitorTilt);
        });
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

    public Command deliverHopperOTM(double timeout) {
        return LoggedCommands.sequence("Deliver hopper OTM",
                LoggedCommands.deadline("SOTM towards prep",
                    LoggedCommands.sequence("Deliver when ready",
                        LoggedCommands.parallel("Wait for vision and movement",
                            LoggedCommands.waitSeconds("Wait for movement to start", 0.10),
                            Vision.instance.ForceVisionUpdate().withTimeout(AutoConstants.visionWaitTime)),
                        LoggedCommands.waitUntil("Wait for alignment", Aiming::isVirtualHubAligned),
                        Superstructure.instance.DeliverToHub().withTimeout(timeout)),
                    LoggedCommands.sequence("Move while shooting",
                        Commands.runOnce(() -> {
                            SwerveAlign.errorReset();
                            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                                SwerveAlign.setTarget(Aiming.aimAtHub().get());
                                return SwerveAlign.getSpeed() * SwerveConstants.maxAngularVelocity;
                            });
                        }),
                        PathCommand("SOTM"),
                        aimOnly()
                    )
                ),
                Swerve.instance.Stop()
            ).finallyDo(PPHolonomicDriveController::clearRotationFeedbackOverride);
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
                    // TODO Emphasize vision over odometry
                    Commands.defer(() -> {
                        Pose2d currentPose = Pose.instance.getPose();
                        Pose2d safePose = Pose.flipIfRed(new Pose2d(6.3, 2.4, Rotation2d.kCCW_90deg));
                        if (optMirrorAuto.get()) {
                            safePose = mirrorPose(safePose);
                        }
                        Transform2d backupTransform = new Transform2d(-0.6, 0, Rotation2d.kZero);
                        Pose2d backupPose = currentPose.transformBy(backupTransform);
                        Pose2d firstPose = new Pose2d(safePose.getX(), backupPose.getY(), safePose.getRotation());

                        return LoggedCommands.sequence("Drive to safe position",
                            new PIDSwerve(Swerve.instance, Pose.instance, backupPose, false, false, PIDSpeed.FAST).withTimeout(3.0),
                            new PIDSwerve(Swerve.instance, Pose.instance, firstPose, false, false, PIDSpeed.FAST),
                            new PIDSwerve(Swerve.instance, Pose.instance, safePose, false, false, PIDSpeed.FAST));
                    }, Set.of(Swerve.instance)),
                    PathCommand("Safe Recovery")),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Swerve.instance.Stop());
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
            deliverHopper(true).withTimeout(2.2),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.sequence("Second pass",
                    sweepPath,
                    Swerve.instance.Stop()),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            deliverHopper(true),
            PathCommand("Shoot - Prep")
        ).finallyDo(Swerve.instance::stopSwerve);

        startingPaths.put(autoBuilderIndirect, firstPathName);

        return autoCommand;
    }

    public Command WatchTilt() {
        return LoggedCommands.waitUntil("Watch Tilt", () -> monitorTilt && Pose.instance.wasTilted());
    }

    public void buildAutos(SendableChooser<Command> chooser) {        
        // Pose2d posePrep = new Pose2d(3.250, 0.600, Rotation2d.kZero);
        // Pose2d posePrepMirror = mirrorPose(posePrep);
        Pose2d poseNext = new Pose2d(3.0, 0.6, Rotation2d.kZero);
        Pose2d poseNextMirror = mirrorPose(poseNext);

        // Command sweep = LoggedCommands.sequence("Default - Double Bump Auto",
        //     LoggedCommands.deadline("Intake through first pass",
        //         LoggedCommands.sequence("First pass",
        //             PathCommand("Trench - Midline - Middle"),
        //             PathCommand("Bump Return - Middle"),
        //             Swerve.instance.Stop()),
        //         Intake.getInstance().RunIntake(),
        //         Shooter.getInstance().Idle()),
        //     deliverHopper(true),
        //     LoggedCommands.deadline("Intake through second pass",
        //         LoggedCommands.sequence("Second pass",
        //             PathCommand("Sweep - Long"),
        //             Swerve.instance.Stop()),
        //         Intake.getInstance().RunIntake(),
        //         Shooter.getInstance().Idle()),
        //     deliverHopper(false)
        // );

        // startingPaths.put(sweep, "Trench - Midline - Middle");
        // addAutoCommand(chooser, sweep);

        Command cmpAuto = LoggedCommands.sequence("CMP Auto",
            MonitorTilt(),
            LoggedCommands.deadline("Intake during scoop",
                LoggedCommands.race("Stay level during scoop",
                    WatchTilt(),
                    PathCommand("Straight Scoop")),
                LoggedCommands.sequence("Deploy and run Intake",
                    LoggedCommands.waitSeconds("Intake drop delay", 0.15),
                    Feeder.getInstance().DeployIntake(),
                    Intake.getInstance().RunIntake()),
                LoggedCommands.sequence("Wait and idle shooter",
                    LoggedCommands.waitSeconds("Shooter idle delay", 0.5),
                    Shooter.getInstance().Idle())),
            Commands.either(
                Recover(),
                Swerve.instance.Stop(),
                Pose.instance::wasTilted),
            deliverHopper(false).withTimeout(AutoConstants.quickShootingTime-0.60),
            Commands.either(
                new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
                new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
                optMirrorAuto
            ),
            MonitorTilt(),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.race("Stay level during sweep",
                    WatchTilt(),
                    PathCommand("CMP Sweep")),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Commands.either(
                Recover(),
                Swerve.instance.Stop(),
                Pose.instance::wasTilted),
            deliverHopper(false).withTimeout(AutoConstants.quickShootingTime+0.60),
            Commands.either(
                new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
                new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
                optMirrorAuto
            ),
            LoggedCommands.deadline("Leave",
                Commands.sequence(
                    PathCommand("SOTM Leave"),
                    Swerve.instance.Stop()
                ),
                Swerve.instance.CoastDriveMotors(),
                Intake.getInstance().RunIntake()
            )
        );

        startingPaths.put(cmpAuto, "Straight Scoop");
        // addAutoCommand(chooser, cmpAuto);
        chooser.setDefaultOption(cmpAuto.getName(), cmpAuto);

        Command cmpAutoSlowed = LoggedCommands.sequence("CMP Auto - Slower Start",
            MonitorTilt(),
            LoggedCommands.deadline("Intake during scoop",
                LoggedCommands.race("Stay level during scoop",
                    WatchTilt(),
                    PathCommand("Straight Scoop - Slowed")),
                LoggedCommands.sequence("Deploy and run Intake",
                    LoggedCommands.waitSeconds("Intake drop delay", 0.15),
                    Feeder.getInstance().DeployIntake(),
                    Intake.getInstance().RunIntake()),
                LoggedCommands.sequence("Wait and idle shooter",
                    LoggedCommands.waitSeconds("Shooter idle delay", 0.5),
                    Shooter.getInstance().Idle())),
            Commands.either(
                Recover(),
                Swerve.instance.Stop(),
                Pose.instance::wasTilted),
            deliverHopper(false).withTimeout(AutoConstants.quickShootingTime-0.60),
            Commands.either(
                new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
                new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
                optMirrorAuto
            ),
            MonitorTilt(),
            LoggedCommands.deadline("Intake through second pass",
                LoggedCommands.race("Stay level during sweep",
                    WatchTilt(),
                    PathCommand("CMP Sweep")),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Commands.either(
                Recover(),
                Swerve.instance.Stop(),
                Pose.instance::wasTilted),
            deliverHopper(false).withTimeout(AutoConstants.quickShootingTime+0.60),
            Commands.either(
                new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
                new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
                optMirrorAuto
            ),
            LoggedCommands.deadline("Leave",
                Commands.sequence(
                    PathCommand("SOTM Leave"),
                    Swerve.instance.Stop()
                ),
                Swerve.instance.CoastDriveMotors(),
                Intake.getInstance().RunIntake()
            )
        );

        startingPaths.put(cmpAutoSlowed, "Straight Scoop");
        addAutoCommand(chooser, cmpAutoSlowed);

        // Command mangoAuto = LoggedCommands.sequence("Mango",
        //     MonitorTilt(),
        //     LoggedCommands.deadline("Intake during disrupt",
        //         LoggedCommands.race("Stay level during disrupt",
        //             WatchTilt(),
        //             PathCommand("Disrupt")),
        //         LoggedCommands.sequence("Deploy and run Intake",
        //             // TODO Uncomment if auto delayed: LoggedCommands.waitSeconds("Intake drop delay", 0.15),
        //             Feeder.getInstance().DeployIntake(),
        //             Intake.getInstance().RunIntake()),
        //         Shooter.getInstance().Idle()),
        //     Commands.either(
        //         Recover(),
        //         Swerve.instance.Stop(),
        //         Pose.instance::wasTilted),
        //     deliverHopper(true),
        //     Commands.either(
        //         new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
        //         new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
        //         optMirrorAuto
        //     ),
        //     MonitorTilt(),
        //     LoggedCommands.deadline("Intake through second pass",
        //         LoggedCommands.race("Stay level during sweep",
        //             WatchTilt(),
        //             PathCommand("CMP Sweep")),
        //         Intake.getInstance().RunIntake(),
        //         Shooter.getInstance().Idle()),
        //     Commands.either(
        //         Recover(),
        //         Swerve.instance.Stop(),
        //         Pose.instance::wasTilted),
        //     deliverHopper(true),
        //     Commands.either(
        //         new PIDSwerve(Swerve.instance, Pose.instance, poseNextMirror, true, false, PIDSpeed.FAST),
        //         new PIDSwerve(Swerve.instance, Pose.instance, poseNext, true, false, PIDSpeed.FAST),
        //         optMirrorAuto
        //     ),
        //     LoggedCommands.deadline("Leave",
        //         Commands.sequence(
        //             PathCommand("SOTM Leave"),
        //             Swerve.instance.Stop()
        //         ),
        //         Swerve.instance.CoastDriveMotors(),
        //         Intake.getInstance().RunIntake()
        //     )
        // );

        // startingPaths.put(mangoAuto, "Disrupt");
        // addAutoCommand(chooser, mangoAuto);

        // Command sotmAuto = LoggedCommands.sequence("SOTM Auto",
        //     // LoggedCommands.deadline("Intake through first pass",
        //     //     LoggedCommands.sequence("First pass",
        //     //         LoggedCommands.race("Stay level through first path",
        //     //             PathCommand("Trench - Midline - Far"),
        //     //             Pose.instance.StayLevel()),
        //     //         PathCommand("Bump Return - Middle"),
        //     //         Swerve.instance.Stop()),
        //     //     LoggedCommands.sequence("Deploy and run Intake",
        //     //         Feeder.getInstance().DeployIntake(),
        //     //         Intake.getInstance().RunIntake()),
        //     //     Shooter.getInstance().Idle()),
        //     LoggedCommands.deadline("Intake during scoop",
        //         PathCommand("Straight Scoop"),
        //         LoggedCommands.sequence("Deploy and run Intake",
        //             Feeder.getInstance().DeployIntake(),
        //             Intake.getInstance().RunIntake()),
        //         Shooter.getInstance().Idle()),
        //     deliverHopperOTM(AutoConstants.firstShootingTime),
        //     Commands.either(
        //         new PIDSwerve(Swerve.instance, Pose.instance, posePrepMirror, true, false, PIDSpeed.FAST),
        //         new PIDSwerve(Swerve.instance, Pose.instance, posePrep, true, false, PIDSpeed.FAST),
        //         optMirrorAuto),
        //     LoggedCommands.deadline("Intake through second pass",
        //         LoggedCommands.sequence("Second pass",
        //             PathCommand("Sweep - Short - from SOTM"),
        //             Swerve.instance.Stop()),
        //         Intake.getInstance().RunIntake(),
        //         Shooter.getInstance().Idle()),
        //     deliverHopperOTM(AutoConstants.quickShootingTime),
        //     Commands.either(
        //         new PIDSwerve(Swerve.instance, Pose.instance, posePrepMirror, true, false, PIDSpeed.FAST),
        //         new PIDSwerve(Swerve.instance, Pose.instance, posePrep, true, false, PIDSpeed.FAST),
        //         optMirrorAuto),
        //     LoggedCommands.deadline("Leave",
        //         Commands.sequence(
        //             PathCommand("SOTM Leave"),
        //             Swerve.instance.Stop()
        //         ),
        //         Intake.getInstance().RunIntake()
        //     )
        // );

        // startingPaths.put(sotmAuto, "Trench - Midline - Far - Slow");
        // addAutoCommand(chooser, sotmAuto);

        Command centerAuto = LoggedCommands.sequence("Center + Depot",
            LoggedCommands.deadline("Backup and get ready",
                LoggedCommands.sequence("Backup",
                    PathCommand("Center"),
                    Swerve.instance.Stop(),
                    LoggedCommands.waitSeconds("Settle", 0.30)),
                LoggedCommands.sequence("Wait then deploy",
                    LoggedCommands.waitSeconds("Wait before deploying", 0.4),
                    Feeder.getInstance().DeployIntake(),
                    Shooter.getInstance().Idle())),
            deliverHopper(false).withTimeout(AutoConstants.firstShootingTime),
            LoggedCommands.deadline("Get fuel from depot",
                PathCommand("Depot Pickup"),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Swerve.instance.Stop(),
            deliverHopper(false));

        startingPaths.put(centerAuto, "Center");
        addAutoCommand(chooser, centerAuto);

        double followDelayDefault = 5.1;
        SmartDashboard.putNumber("Autos/Follow Delay", followDelayDefault);
        Command followAuto = LoggedCommands.sequence("Follow across hub",
            LoggedCommands.deadline("Starting sequence",
                Commands.defer(() -> LoggedCommands.waitSeconds("Wait before following", SmartDashboard.getNumber("Autos/Follow Delay", followDelayDefault)), Set.of()),
                LoggedCommands.sequence("Starting sequence",
                    LoggedCommands.deadline("Backup to shoot",
                        LoggedCommands.sequence("Backup",
                            PathCommand("Bump - Shoot"),
                            Swerve.instance.Stop()),
                        LoggedCommands.sequence("Wait and deploy Intake",
                            LoggedCommands.waitSeconds("Intake drop delay", 0.15),
                            Intake.getInstance().RunIntake()),
                        LoggedCommands.sequence("Wait and idle shooter",
                            LoggedCommands.waitSeconds("Shooter idle delay", 0.3),
                            Shooter.getInstance().Idle())),
                    deliverHopper(false).withTimeout(1.7),
                    PathCommand("Shoot - Prep"),
                    Swerve.instance.Stop(),
                    LoggedCommands.idle("Wait for time to expire"))),
            LoggedCommands.deadline("Get fuel from along hub",
                PathCommand("Across Hub"),
                Intake.getInstance().RunIntake(),
                Shooter.getInstance().Idle()),
            Swerve.instance.Stop(),
            // deliverHopper(false).withTimeout(AutoConstants.quickShootingTime),
            // LoggedCommands.deadline("Get fuel from depot",
            //     PathCommand("Depot Pickup"),
            //     Intake.getInstance().RunIntake(),
            //     Shooter.getInstance().Idle()),
            // Swerve.instance.Stop(),
            deliverHopper(false));

        startingPaths.put(followAuto, "Bump - Shoot");
        addAutoCommand(chooser, followAuto);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public Command SOTMTest() {
        Pose2d SOTMStart = new Pose2d(2.75, 2.30, Rotation2d.fromDegrees(45));

        return LoggedCommands.sequence("SOTM Test",
            new PIDSwerve(Swerve.instance, Pose.instance, SOTMStart, true, true),
            Swerve.instance.Stop(),
            deliverHopperOTM(Constants.hopperShootingTime),
            Swerve.instance.Stop()
        );
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

        DogLog.log("Autos/Starting Pose Error", poseDifference);
        DogLog.log("Autos/Starting Pose OK", differenceOK);
        DogLog.log("Autos/Red Alliance?", Robot.isRed());
    }
}