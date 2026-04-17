package frc.robot.commands.pidswerve;

import frc.robot.subsystems.swerve.SwerveConstants;

public class PIDSwerveConstants {
    public static final double translationKP = 0.05;
    public static final double roughTranslationKP = 0.10;
    public static final double positionTolerance = 4.0; // inches
    public static final double roughPositionTolerance = 8.0; // inches
    public static final double roughAngleTolerance = 8.0; // degrees
    public static final double positionKS = 0.02;
    public static final double positionIZone = 6.0;
   
    public static final double maxAngularVelocity = SwerveConstants.maxAngularVelocity / 2.0;

    public static final double objRotationKP = 0.013;
    public static final double objRotationKD = 0.00;

    public static final double distanceAfterLock = 0.4; // Meters to drive after locking the object
    public static final double lockPitch = 7.5; // How close to get to lock
    public static final double lockYaw = 8.5; // How aligned to be to lock
    public static final double objSeekSpeed = 0.20; // Speed when pursuing the object    

    public enum PIDSpeed {
        SLOW(SwerveConstants.maxSpeed / 6.0),
        FAST(SwerveConstants.maxSpeed / 3.0),
        TURBO(SwerveConstants.maxSpeed / 1.5);
        
        PIDSpeed(double speed) {
            this.speed = speed;
        }

        public double speed;
    }
}
