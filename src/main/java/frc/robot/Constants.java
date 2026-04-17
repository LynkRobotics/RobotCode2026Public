package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
    public static final boolean atHQ = true;
    public static final boolean fullDashboard = true;
    public static final boolean realtimePriority = true;
    public static final boolean profileTime = true;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double stickDeadband = 0.03;
    public static final double driveStickSensitivity = 1.00; 
    public static final double turnStickSensitivity = 1.00;
    public static final double aimingOverride = 0.001;
    public static final double mechanismSlowdown = 1.0; // Useful to help analyze issues with mechanisms
    public static final double hopperShootingTime = 3.5; // How many seconds expected to basically unload a full hopper
    public final double rotSpeedLimit = 0.70; // Max percentage of rotation speed to use when remotely controlled
    // TODO Should this be turnStickSensitivity instead

    // Elastic Notifications
    public static final int warningTime = 4000;
    public static final int errorTime = 7000;   
}