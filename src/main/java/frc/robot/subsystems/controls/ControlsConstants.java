package frc.robot.subsystems.controls;

public class ControlsConstants {
    public enum ControlMode {
        DEFAULT,
        TEST,
        SWERVE_SYSID_DRIVE,
        SWERVE_SYSID_STEER,
        FLYWHEEL_SYSID,
    }

    public static final ControlMode controlMode = ControlMode.DEFAULT;
}