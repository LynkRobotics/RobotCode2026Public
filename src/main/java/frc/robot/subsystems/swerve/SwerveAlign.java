package frc.robot.subsystems.swerve;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.pose.Pose;

// Separate class for swerve alignment constants and calculation
public class SwerveAlign {
    // Rotation PID for auto-aligning
    public static final PIDController rotationPID = new PIDController(0.0099, 0.0, .0);
    public static final double rotationKS = 0.018;
    public static final double rotationMax = 0.40;
    public static final double rotationIZone = 2.0; // degrees

    static {
        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setIZone(rotationIZone); // Only use Integral term within this range
        rotationPID.setIntegratorRange(rotationKS * -2, rotationKS * 2);
        rotationPID.reset();
    }

    public static void errorReset() {
        errorReset(rotationPID);
    }

    public static void errorReset(PIDController pid) {
        pid.reset();
        DogLog.log("SwerveAlign/Last Reset", Timer.getTimestamp());
    }

    public static void setTarget(Rotation2d targetAngle) {
        setTarget(targetAngle, rotationPID);
    }

    public static void setTarget(Rotation2d targetAngle, PIDController pid) {
        pid.setSetpoint(targetAngle.getDegrees());
        DogLog.log("SwerveAlign/Target Angle", targetAngle.getDegrees());
    }

    public static double getSpeed() {
        return getSpeed(Pose.instance.getPose().getRotation());
    }

    public static double getSpeed(Rotation2d currentAngle) {
        return getSpeed(currentAngle, rotationPID);
    }

    // TODO Unify with PID Swerve logic
    public static double getSpeed(Rotation2d currentAngle, PIDController pid) {
        double currentDeg = currentAngle.getDegrees();
        double correction = pid.calculate(currentDeg);
        double feedForward = rotationKS * Math.signum(correction);
        double output = MathUtil.clamp(correction + feedForward, -rotationMax, rotationMax);

        DogLog.log("SwerveAlign/Current Angle", currentDeg);
        DogLog.log("SwerveAlign/PID correction", correction);
        DogLog.log("SwerveAlign/Feedforward", feedForward);
        DogLog.log("SwerveAlign/Output", output);
        
        return output;
    }

    public static boolean aligned() {
        return aligned(rotationPID);
    }

    public static boolean aligned(double tolerance) {
        return aligned(rotationPID, tolerance);
    }

    public static boolean aligned(PIDController pid) {
        return pid.atSetpoint();
    }

    public static boolean aligned(PIDController pid, double tolerance) {
        return Math.abs(pid.getError()) < tolerance;
    }
}