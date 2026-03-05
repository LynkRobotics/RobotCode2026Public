package frc.robot.commands;

import frc.lib.util.LoggedCommand;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.pose.Pose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveAlign;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Options.*;

public class TeleopSwerve extends LoggedCommand {
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private DoubleSupplier speedLimitSupplier;
    private Supplier<Optional<Rotation2d>> autoAimSupplier = null;
    private boolean autoAiming = false;
    private Rotation2d lastAngle = Rotation2d.kZero;
    public static final PIDController pacmanPID = new PIDController(0.085, 0.0, .0);
    private final double rotSpeedLimit = 0.70;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedLimitSupplier) {
        super();

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedLimitSupplier = speedLimitSupplier;
    }

    public void setAutoAimSupplier(Supplier<Optional<Rotation2d>> supplier) {
        this.autoAimSupplier = supplier;
    }

    @Override
    public void execute() {
        super.execute();

        if (optServiceMode.get()) {
            return;
        }
        
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // TODO Get *every* time?
        double teleOpMult = SmartDashboard.getNumber("TeleOp Speed Governor", 1.0);
        
        // Use exponential controls
        // if (translationExpo != 1.0) {
        //     translationVal = Math.abs(Math.pow(Math.abs(translationVal), translationExpo)) * Math.signum(translationVal);
        //     strafeVal = Math.abs(Math.pow(Math.abs(strafeVal), translationExpo)) * Math.signum(strafeVal);
        // }
        // double rotationExpo = 3.5;
        // if (rotationExpo != 1.0) {
        //     rotationVal = Math.abs(Math.pow(Math.abs(rotationVal), rotationExpo)) * Math.signum(rotationVal);
        // }
        translationVal = translationVal * translationVal * Math.signum(translationVal);
        strafeVal = strafeVal * strafeVal * Math.signum(strafeVal);
        rotationVal = rotationVal * rotationVal * Math.signum(rotationVal);

        translationVal *= teleOpMult;
        strafeVal *= teleOpMult;
        rotationVal *= teleOpMult;
        if (optLimitRotation.get()) {
            rotationVal *= rotSpeedLimit;
        }

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        boolean holdAngle = false;
        if (Math.abs(rotationVal) < Constants.aimingOverride) {
            if (optPacManMode.get() && (Math.abs(translationVal) > Constants.aimingOverride || Math.abs(strafeVal) > Constants.aimingOverride)) {
                Rotation2d angle = new Rotation2d(translationVal, strafeVal);
                final double maxAngle = 135;
                final double pacmanRotMax = 0.25;
                Rotation2d currentAngle = Pose.instance.getHeading();
                Rotation2d angleDiff = angle.minus(currentAngle);
                double angleDiffDeg = MathUtil.inputModulus(angleDiff.getDegrees(), -180, 180);

                if (Math.abs(angleDiffDeg) <= maxAngle) {
                    if (!autoAiming) {
                        SwerveAlign.errorReset(pacmanPID);
                        autoAiming = true;
                    }
                    SwerveAlign.setTarget(angle, pacmanPID); // Constantly update target
                    rotationVal = SwerveAlign.getSpeed(currentAngle, pacmanPID);
                    rotationVal = MathUtil.clamp(rotationVal, -pacmanRotMax, pacmanRotMax);
                    autoAiming = true;
                } else {
                    autoAiming = false;
                }
            } else if (autoAimSupplier != null) {
                Optional<Rotation2d> optAngle = autoAimSupplier.get();
                Rotation2d angle = Rotation2d.kZero;
                boolean prevAutoAiming = autoAiming;
                if (optAngle.isPresent()) {
                    angle = optAngle.get();
                    autoAiming = true;
                } else if (optHoldAngle.get()) {
                    angle = lastAngle;
                    autoAiming = true;
                    holdAngle = true;
                } else {
                    autoAiming = false;
                }
                if (autoAiming) {
                    if (!prevAutoAiming) {
                        SwerveAlign.errorReset();
                    }
                    SwerveAlign.setTarget(angle); // Constantly update target
                    rotationVal = SwerveAlign.getSpeed();
                }
            } else {
                autoAiming = false;
            }
        }

        if (!holdAngle) {
            lastAngle = Pose.instance.getHeading();
        }

        /* Drive */
        s_Swerve.drive(
            // TODO SwerveConstants.slowMode while intaking?
            new Translation2d(translationVal, strafeVal).times(speedLimitSupplier.getAsDouble()).times(SwerveConstants.maxSpeed),
            rotationVal * SwerveConstants.maxAngularVelocity * speedLimitSupplier.getAsDouble(),
            true
        );
    }
}