// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.lib.math.Hysteresis;
import frc.robot.Aiming;
import frc.robot.Field.FieldSide;
import frc.robot.Field.TrenchStatus;
import frc.robot.Field.Zone;
import frc.robot.FieldConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.autos.AutoConstants;

import static frc.robot.Field.trenchRunHalfLength;
import static frc.robot.Field.zoneHysteresis;
import static frc.robot.Field.trenchHysteresis;

public class Pose extends SubsystemBase {
    public static final Pose instance = new Pose();

    private int dashboardCounter = 0;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pigeon2 gyro;
    private FieldSide fieldSide = FieldSide.OUTPOST;
    private Zone zone = Zone.ALLIANCE;
    private TrenchStatus trenchStatus = TrenchStatus.IN_RUN;

    public Pose() {
        gyro = new Pigeon2(Ports.PIGEON.id, Ports.PIGEON.bus);
        gyro.getConfigurator().apply(new Pigeon2Configuration().withMountPose(PoseConstants.gyroMountPose));
        gyro.setYaw(0);

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getGyroYaw(), Swerve.instance.getModulePositions(), new Pose2d());
        Vision.setPoseEstimator(poseEstimator);
        Vision.setHeadingProvider(this::getHeading);

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            Swerve.instance::getSpeeds,
            (speeds, feedforwards) -> Swerve.instance.driveRobotRelativeAuto(speeds),
            // TODO Configure PIDs
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(4.5, 0.0, 0.0)  // Rotation PID constants
            ),
            AutoConstants.robotConfig,
            Robot::isRed,
            Swerve.instance // Reference to Swerve subsystem to set requirements
        );

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            DogLog.log("Pose/Auto Target Pose", targetPose);
        });
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            DogLog.log("Pose/Active Path", activePath.toArray(Pose2d[]::new)); //we have to convert the List of poses PathPlanner gives us to an array because DogLog does not support list, fourtunetely aScope doesn't care whether its a list or an array
        });
        PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
            DogLog.log("Pose/PP Current Pose", currentPose);
        });

        // SmartDashboard.putData("Pose/Heading", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("Gyro");
        //         builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
        //     }
        // });

        // SmartDashboard.putData("Pose/Reef Bearing", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("Gyro");
        //         builder.addDoubleProperty("Value", () -> reefBearing(getPose().getTranslation()).getDegrees(), null);
        //     }
        // });

        // SmartDashboard.putNumber("Pose/Rot KP", PoseConstants.rotationPID.getP());
        // SmartDashboard.putNumber("Pose/Rot KD", PoseConstants.rotationPID.getD());
        // SmartDashboard.putData("Pose/Set Rot PID", LoggedCommands.runOnce("Set Pose Rotation PID", () ->
        //     { PoseConstants.rotationPID.setPID(SmartDashboard.getNumber("Pose/Rot KP", 0.0), 0.0,
        //         SmartDashboard.getNumber("Pose/Rot KD", 0.0));
        //     }).ignoringDisable(true));

        // SmartDashboard.putNumber("Pose/Rot KS", PoseConstants.rotationKS);
        // SmartDashboard.putData("Pose/Rot @ KS", LoggedCommands.startEnd("Rot @ KS",
        //     () -> Swerve.instance.drive(new Translation2d(0, 0),
        //         SmartDashboard.getNumber("Pose/Rot KS", 0.0) * PIDSwerveConstants.maxAngularVelocity,
        //         true),
        //         Swerve.instance::Stop, Swerve.instance));
    }

    public static String prettyPose(Pose2d pose) {
        return String.format("(%01.2f, %01.2f @ %01.1f)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
    public Rotation2d getGyroYaw() {
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public Rotation2d getGyroRoll() {
        return new Rotation2d(gyro.getRoll().getValue());
    }

    public Rotation2d getGyroPitch() {
        return new Rotation2d(gyro.getPitch().getValue());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Yaw");
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), Swerve.instance.getModulePositions(), pose);
        DogLog.log("Pose/Status/Setting Pose", pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), Swerve.instance.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        setHeading(new Rotation2d());
        DogLog.log("Pose/Gyro/Status", "Zeroed Gyro Heading");
    }

    public void resetHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        } else {
            setHeading(new Rotation2d());
        }
    }

    public static Translation2d flipIfRed(Translation2d position) {
        return Robot.isRed() ? FlippingUtil.flipFieldPosition(position) : position;
    }

    public static Pose2d flipIfRed(Pose2d pose) {
        return Robot.isRed() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Rotation2d flipIfRed(Rotation2d rotation) {
        return Robot.isRed() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
    }

    public static double distanceTo(Translation2d target) {
        return Pose.instance.getPose().getTranslation().getDistance(target);
    }

    public double visionDifference() {
        Pose2d visionPose = Vision.instance.lastPose();

        if (visionPose == null) {
            return Double.POSITIVE_INFINITY;
        }

        return getPose().getTranslation().getDistance(visionPose.getTranslation());
    }

    public static Translation2d hubCenter() {
        return flipIfRed(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    }

    public double hubDistance() {
        return hubDistance(getPose().getTranslation());
    }

    public double hubDistance(Translation2d position) {
        return position.getDistance(hubCenter());
    }

    public double passDistance() {
        return passDistance(getPose().getTranslation());
    }

    public double passDistance(Translation2d position) {
        return position.getDistance(Aiming.passingLocation());
    }

    public static Rotation2d bearing(Translation2d from, Translation2d to) {
        return to.minus(from).getAngle();
    }

    public Zone getZone() {
        return zone;
    }

    public FieldSide getFieldSide() {
        return fieldSide;
    }

    public TrenchStatus getTrenchStatus() {
        return trenchStatus;
    }

    private boolean insideBumpY(double blueY) {
        return blueY > FieldConstants.LinesHorizontal.rightBumpEnd && blueY < FieldConstants.LinesHorizontal.leftBumpStart;
    }

    public TrenchStatus getTrenchStatus(Translation2d bluePosition) {
        if (!insideBumpY(bluePosition.getY())) {
            double x = bluePosition.getX();
            
            if (x < FieldConstants.LinesVertical.hubCenter) {
                return Hysteresis.calculate(x, FieldConstants.LinesVertical.hubCenter - trenchRunHalfLength, trenchHysteresis, trenchStatus, TrenchStatus.OUT_RUN, TrenchStatus.IN_RUN);
            } else if (x < FieldConstants.LinesVertical.hubCenter + trenchRunHalfLength + trenchHysteresis) {
                return Hysteresis.calculate(x, FieldConstants.LinesVertical.hubCenter + trenchRunHalfLength, trenchHysteresis, trenchStatus, TrenchStatus.IN_RUN, TrenchStatus.OUT_RUN);
            } else if (x < FieldConstants.LinesVertical.oppHubCenter) {
                return Hysteresis.calculate(x, FieldConstants.LinesVertical.oppHubCenter - trenchRunHalfLength, trenchHysteresis, trenchStatus, TrenchStatus.OUT_RUN, TrenchStatus.IN_RUN);
            } else if (x < FieldConstants.LinesVertical.oppHubCenter + trenchRunHalfLength + trenchHysteresis) {
                return Hysteresis.calculate(x, FieldConstants.LinesVertical.oppHubCenter + trenchRunHalfLength, trenchHysteresis, trenchStatus, TrenchStatus.IN_RUN, TrenchStatus.OUT_RUN);
            }
        }

        return TrenchStatus.OUT_RUN;
    }

    public Zone getZone(Translation2d bluePosition) {
        double x = bluePosition.getX();
        final double transitionWidth = 0.60;
        double transitionStart = FieldConstants.LinesVertical.allianceZone - transitionWidth;

        if (x < transitionStart + zoneHysteresis) {
            return Hysteresis.calculate(x, transitionStart, zoneHysteresis, zone, Zone.ALLIANCE, Zone.TRANSITION);
        } else {
            return Hysteresis.calculate(x, FieldConstants.LinesVertical.hubCenter, zoneHysteresis, zone, Zone.TRANSITION, Zone.REMOTE);
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), Swerve.instance.getModulePositions());
        Pose2d pose = getPose();
        Translation2d position = pose.getTranslation();
        Translation2d bluePosition = flipIfRed(position);

        if (dashboardCounter++ >= PoseConstants.dashboardInterval) {
            Robot.field.setRobotPose(pose);
            dashboardCounter = 0;
        }
        
        fieldSide = Hysteresis.calculate(bluePosition.getY(), FieldConstants.LinesHorizontal.center, PoseConstants.fieldSideBuffer.in(Units.Meters) / 2.0, fieldSide, FieldSide.OUTPOST, FieldSide.DEPOT);
        trenchStatus = getTrenchStatus(bluePosition);
        zone = getZone(bluePosition);
        
        DogLog.log("Pose/Pose", pose);
        DogLog.log("Pose/Field Side", fieldSide);
        DogLog.log("Pose/Zone", zone);
        DogLog.log("Pose/Trench Status", trenchStatus);
        DogLog.log("Pose/Gyro/Heading", getHeading().getDegrees());
        DogLog.log("Pose/Gyro/Raw Yaw", getGyroYaw());
        DogLog.log("Pose/Hub Pose", hubCenter());
        DogLog.log("Pose/Hub Distance", hubDistance(position));
        SmartDashboard.putNumber("Pose/Hub Distance", hubDistance(position));

        Aiming.logAll();
    }
}