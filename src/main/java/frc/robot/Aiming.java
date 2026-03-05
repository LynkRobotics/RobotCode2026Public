package frc.robot;

import static frc.robot.Options.optAutoAiming;
import static frc.robot.Options.optTestAiming;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Field.FieldSide;
import frc.robot.Field.PassTarget;
import frc.robot.Field.TrenchStatus;
import frc.robot.Field.Zone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.superstructure.Superstructure;

// Logic to determine what to aim at, when
public class Aiming {
    public static final Pose poseSystem = Pose.instance;
    public static final Intake intakeSystem = Intake.getInstance();
    private static PassTarget passTarget = PassTarget.CORNER;
    private static final double shootingAlignmentTolerance = 1.0; // degrees
    private static final double passingAlignmentTolerance = 2.0; // degrees

    public static Translation2d hubLocation() {
        // Translation2d direction = Swerve.instance.direction();

        return Pose.hubCenter();
    }
    
    public static Rotation2d hubAngle() {
        return hubAngle(poseSystem.getPose().getTranslation());
    }

    public static Rotation2d hubAngle(Translation2d position) {
        return Pose.bearing(position, hubLocation());
    }

    public static boolean isHubAligned() {
        return isHubAligned(poseSystem.getPose());
    }

    public static boolean isHubAligned(Pose2d pose) {
        return Math.abs(hubAngle(pose.getTranslation()).minus(pose.getRotation()).getDegrees()) < shootingAlignmentTolerance;
    }

    public static boolean isPassAligned() {
        Translation2d passingLocation = passingLocation();
        Pose2d pose = poseSystem.getPose();
        Rotation2d bearing = Pose.bearing(pose.getTranslation(), passingLocation);

        return isPassAligned(pose.getRotation(), bearing);
    }

    public static boolean isPassAligned(Rotation2d rotation, Rotation2d bearing) {

        return Math.abs(rotation.minus(bearing).getDegrees()) < passingAlignmentTolerance;
    }

    public static void setPassTarget(PassTarget target) {
        passTarget = target;
    }

    public static PassTarget getPassTarget() {
        return passTarget;
    }

    public static Translation2d passingLocation() {
        return passingLocation(passTarget, poseSystem.getFieldSide());
    }

    private static Translation2d passingLocation(PassTarget target, FieldSide fieldSide) {
        Translation2d position = target.position;

        if (fieldSide == FieldSide.DEPOT) {
            position = new Translation2d(position.getMeasureX(), Field.width.minus(position.getMeasureY()));
        }

        return Pose.flipIfRed(position);
    }

    public static Rotation2d bumpAngle() {
        return bumpAngle(poseSystem.getPose().getRotation());
    }

    public static Rotation2d bumpAngle(Rotation2d angle) {
        double degrees = MathUtil.inputModulus(angle.getDegrees(), -180, 180);
        double absDegrees = Math.abs(degrees);
        double degreesOff = absDegrees - 90;
        double degreesMin = 30;

        if (Math.abs(degreesOff) >= degreesMin) {
            return angle;
        }
        return Rotation2d.fromDegrees((90 + degreesMin * Math.signum(degreesOff)) * Math.signum(degrees));
    }

    public static Rotation2d trenchAngle() {
        return trenchAngle(poseSystem.getPose().getRotation());
    }

    public static Rotation2d trenchAngle(Rotation2d angle) {
        double deg = angle.getDegrees();

        if (deg < 90 && deg > -90) {
            return Rotation2d.kZero;
        } else {
            return Rotation2d.k180deg;
        }
    }

    public static final Optional<Rotation2d> autoAim() {
        if (optTestAiming.get()) {
            return Optional.of(Rotation2d.fromDegrees(SmartDashboard.getNumber("Aiming/Test Angle", 0)));
        }

        if (intakeSystem.getMode() == IntakeMode.ACTIVE) {
            return Optional.empty();
        }

        boolean autoAim = Shooter.getInstance().isActive();

        if (!autoAim && poseSystem.getTrenchStatus() == TrenchStatus.IN_RUN && optAutoAiming.get()) {
            return Optional.of(trenchAngle());
        }

        Zone zone = poseSystem.getZone();
        if (!autoAim && optAutoAiming.get() && zone == Zone.ALLIANCE) {
            autoAim = Superstructure.instance.hopperFull();
        }

        if (autoAim) {
            if (zone == Zone.REMOTE) {
                return Optional.of(Pose.bearing(poseSystem.getPose().getTranslation(), passingLocation()));
            }
            return aimAtHub();
        }

        return Optional.empty();
    }

    public static final Optional<Rotation2d> aimAtHub() {
        return Optional.of(hubAngle());
    }

    public static void logAll() {
        FieldSide fieldSide = poseSystem.getFieldSide();
        Optional<Rotation2d> autoAim = autoAim();

        DogLog.log("Aiming/Pass target", passTarget.name());
        DogLog.log("Aiming/Pass target position", passingLocation());
        DogLog.log("Aiming/Pass target distance", poseSystem.getPose().getTranslation().getDistance(passingLocation()));
        DogLog.log("Aiming/Pass Aligned", isPassAligned());
        DogLog.log("Aiming/Corner target", passingLocation(PassTarget.CORNER, fieldSide));
        DogLog.log("Aiming/Trench target", passingLocation(PassTarget.TRENCH, fieldSide));
        DogLog.log("Aiming/Middle target", passingLocation(PassTarget.MIDDLE, fieldSide));
        DogLog.log("Aiming/Auto angle", autoAim.orElse(null));
        DogLog.log("Aiming/Hub Angle", hubAngle().getDegrees());
        DogLog.log("Aiming/Hub Aligned", isHubAligned());

        SmartDashboard.putString("Aiming/Pass target", passTarget.toString());
    }
}