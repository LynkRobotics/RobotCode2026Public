package frc.robot;

import static frc.robot.Options.optAutoAiming;
import static frc.robot.Options.optSOTM;
import static frc.robot.Options.optTestAiming;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Field.FieldSide;
import frc.robot.Field.PassTarget;
import frc.robot.Field.TrenchStatus;
import frc.robot.Field.Zone;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.superstructure.Superstructure;

// Logic to determine what to aim at, when
public class Aiming {
    public static final Pose poseSystem = Pose.instance;
    public static final Intake intakeSystem = Intake.getInstance();
    private static PassTarget passTarget = PassTarget.CORNER;
    private static final double shootingAlignmentTolerance = Units.inchesToMeters(5.0);
    private static final double passingAlignmentTolerance = Units.inchesToMeters(18.0);
    // private static final double shootingAlignmentToleranceFixed = 1.0; // degrees;
    private static final double passingAlignmentToleranceFixed = 2.0; // degrees;

    public static final double maxSOTMInterations = 3;
    public static final double convergenceTolerance = Units.inchesToMeters(1.5);

    public static double tolerance(double distance, double maxError) {
        double tolerance = Math.toDegrees(2 * Math.asin(maxError / (2 * distance)));
        
        DogLog.log("Aiming/Tolerance", tolerance);
        return tolerance;
    }

    public static double shootingTolerance(double distance) {
        return tolerance(distance, shootingAlignmentTolerance);
    }

    public static double passingTolerance(double distance) {
        return tolerance(distance, passingAlignmentTolerance);
    }

    public static Translation2d getVirtualTarget(Translation2d target) {
        return getVirtualTarget(target, poseSystem.getPose().getTranslation());
    }

    public static Translation2d getVirtualTarget(Translation2d target, Translation2d position) {
        if (!optSOTM.get()) {
            return target;
        }

        Translation2d movement = Swerve.instance.direction();
        Translation2d virtualTarget = target;
        Translation2d lastTarget;

        // Run the calculation iteratively 3 times to converge on the exact target
        for (int i = 0; i < maxSOTMInterations; i++) {
            // 1. Find distance to the CURRENT estimate of the virtual target
            double distanceToVirtualTarget = position.getDistance(virtualTarget);

            // 2. Look up the estimated time of flight for this distance
            double timeOfFlight = ShooterConstants.distanceToToF.get(distanceToVirtualTarget);
            double fudgeFactor = 0.65;
            timeOfFlight *= fudgeFactor;

            // 3. Create a vector representing the robot's movement during the shot's flight
            Translation2d robotMovementDuringFlight = new Translation2d(movement.getX() * timeOfFlight, movement.getY() * timeOfFlight);

            // 4. Subtract the robot's movement from the REAL target location
            lastTarget = virtualTarget;
            virtualTarget = target.minus(robotMovementDuringFlight);

            // Abort if we're close enough to converging
            if (Math.abs(virtualTarget.getX() - lastTarget.getX()) < convergenceTolerance && Math.abs(virtualTarget.getY() - lastTarget.getY()) < convergenceTolerance) {
                break;
            }
        }

        return virtualTarget;
    }

    public static Translation2d virtualHubLocation() {
        return virtualHubLocation(poseSystem.getPose().getTranslation());
    }

    public static Translation2d virtualHubLocation(Translation2d position) {
        return getVirtualTarget(Pose.hubCenter(), position);
    }

    public static double virtualHubDistance(Translation2d position) {
        return position.getDistance(virtualHubLocation(position));
    }

    public static double virtualHubDistance() {
        return virtualHubDistance(poseSystem.getPose().getTranslation());
    }

    public static Translation2d hubLocation() {
        return Pose.hubCenter();
    }

    public static Rotation2d hubAngle() {
        return hubAngle(poseSystem.getPose().getTranslation());
    }

    public static Rotation2d hubAngle(Translation2d position) {
        return Pose.bearing(position, hubLocation());
    }

    public static Rotation2d virtualHubAngle() {
        return virtualHubAngle(poseSystem.getPose().getTranslation());
    }

    public static Rotation2d virtualHubAngle(Translation2d position) {
        return Pose.bearing(position, virtualHubLocation(position));
    }

    public static boolean isHubAligned() {
        return isHubAligned(poseSystem.getPose());
    }

    public static boolean isHubAligned(Pose2d pose) {
        double distance = pose.getTranslation().getDistance(hubLocation());
        double tolerance = shootingTolerance(distance);

        return Math.abs(hubAngle(pose.getTranslation()).minus(pose.getRotation()).getDegrees()) < tolerance;
    }

    public static boolean isVirtualHubAligned() {
        return isVirtualHubAligned(poseSystem.getPose());
    }

    public static boolean isVirtualHubAligned(Pose2d pose) {
        Translation2d position = pose.getTranslation();
        Translation2d target = virtualHubLocation(position);

        return isVirtualHubAligned(pose, target);
    }

    public static boolean isVirtualHubAligned(Pose2d pose, Translation2d target) {
        Translation2d position = pose.getTranslation();
        double distance = position.getDistance(target);
        double tolerance = shootingTolerance(distance);

        return Math.abs(Pose.bearing(position, target).minus(pose.getRotation()).getDegrees()) < tolerance;
    }

    public static boolean isPassAligned() {
        Translation2d passingLocation = passingLocation();
        Pose2d pose = poseSystem.getPose();
        Rotation2d bearing = Pose.bearing(pose.getTranslation(), passingLocation);

        return isPassAligned(pose.getRotation(), bearing);
    }

    public static boolean isPassAligned(Rotation2d rotation, Rotation2d bearing) {

        // TODO Call passingTolerance()
        return Math.abs(rotation.minus(bearing).getDegrees()) < passingAlignmentToleranceFixed;
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

        boolean autoAim = (Shooter.getInstance().isActive() && Superstructure.instance.automaticShot());

        if (!autoAim && poseSystem.getTrenchStatus() == TrenchStatus.IN_RUN && optAutoAiming.get()) {
            return Optional.of(trenchAngle());
        }

        Zone zone = poseSystem.getZone();
        if (!autoAim && optAutoAiming.get() && zone == Zone.ALLIANCE) {
            autoAim = Superstructure.instance.hopperFull();
        }

        if (autoAim) {
            Translation2d position = poseSystem.getPose().getTranslation();

            if (zone == Zone.REMOTE) {
                return Optional.of(Pose.bearing(position, getVirtualTarget(passingLocation(), position)));
            }
            return Optional.of(virtualHubAngle(position));
        }

        return Optional.empty();
    }

    public static final Optional<Rotation2d> aimAtHub() {
        // NOTE: Presumes no robot movement, so only use if stationary or for testing
        return Optional.of(hubAngle());
    }

    public static void logAll() {
        FieldSide fieldSide = poseSystem.getFieldSide();
        Optional<Rotation2d> autoAim = autoAim();
        Pose2d pose = poseSystem.getPose();
        Translation2d position = pose.getTranslation();

        DogLog.log("Aiming/Pass target", passTarget.name());
        DogLog.log("Aiming/Pass target position", passingLocation());
        DogLog.log("Aiming/Pass target distance", position.getDistance(passingLocation()));
        DogLog.log("Aiming/Pass Aligned", isPassAligned());
        DogLog.log("Aiming/Corner target", passingLocation(PassTarget.CORNER, fieldSide));
        DogLog.log("Aiming/Trench target", passingLocation(PassTarget.TRENCH, fieldSide));
        DogLog.log("Aiming/Middle target", passingLocation(PassTarget.MIDDLE, fieldSide));
        DogLog.log("Aiming/Auto angle", autoAim.isPresent() ? autoAim.get().getDegrees() : -999);
        DogLog.log("Aiming/Real Hub Angle", hubAngle(position).getDegrees());

        Translation2d virtualHub = virtualHubLocation();
        DogLog.log("Aiming/Virtual Hub Location", virtualHub);
        DogLog.log("Aiming/Virtual Hub Angle", virtualHubAngle(position).getDegrees());
        DogLog.log("Aiming/Virtual Hub Aligned", isVirtualHubAligned(pose, virtualHub));

        SmartDashboard.putString("Aiming/Pass target", passTarget.toString());
    }
}