package frc.robot.subsystems.pose;

import com.ctre.phoenix6.configs.MountPoseConfigs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class PoseConstants {
    public static final int dashboardInterval = 20;

    public static final MountPoseConfigs gyroMountPose = new MountPoseConfigs().withMountPoseYaw(-180).withMountPosePitch(0).withMountPoseRoll(180);

    // Robot dimensions
    public static final Distance robotFrameLength = Units.Inches.of(26.0);
    public static final Distance robotFrameWidth = Units.Inches.of(26.0);
    public static final Distance bumperWidth = Units.Inches.of(2.75);
    public static final Distance centerToFrontBumper = robotFrameLength.div(2.0).plus(bumperWidth);

    public static final Distance fieldSideBuffer = Units.Meters.of(1.0);
}