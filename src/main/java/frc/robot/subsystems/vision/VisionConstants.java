package frc.robot.subsystems.vision;

import java.util.EnumMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class VisionConstants {
    public static final int dashboardInterval = 20;
    public static final double fieldBorderMargin = 0.25; // Reject poses this far outside the field
    public static final double maxZError = 0.5; // Reject poses this far above or below the floor
    public static final double autoAcceptAmbiguity = 0.1; // Automatically accept results with ambiguity less than this
    public static final double maxAmbiguity = 0.35; // Reject results with ambiguity greater than this
    public static final double odometryUnreliableMultiplier = 0.2; // Multiplier for standard deviation when odometry is unreliable (e.g. robot is tilted)

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.10; // Meters
    public static double angularStdDevBaseline = 2.0; // Radians

    public enum Camera {
        FRONTLEFT("FrontLeftCam", new Transform3d(
            new Translation3d(Units.Inches.of(-10.299), Units.Inches.of(12.250), Units.Inches.of(20.579)), 
            new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(-27.5), Units.Degree.of(0.0)))),
        SIDELEFT("SideLeftCam", new Transform3d(
            new Translation3d(Units.Inches.of(-9.488), Units.Inches.of(11.940), Units.Inches.of(13.529)), 
            new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(0.0), Units.Degree.of(90.0)))),
        FRONTRIGHT("FrontRightCam", new Transform3d(
            new Translation3d(Units.Inches.of(-10.299), Units.Inches.of(-12.250), Units.Inches.of(20.579)), 
            new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(-27.5), Units.Degree.of(0.0)))),
        SIDERIGHT("SideRightCam", new Transform3d(
            new Translation3d(Units.Inches.of(-9.488), Units.Inches.of(-11.940), Units.Inches.of(13.529)), 
            new Rotation3d(Units.Degree.of(0.0), Units.Degree.of(0.0), Units.Degree.of(-90.0))));
    
        public final String name;
        public final Transform3d robotToCamera;
            
        Camera(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    public static final Camera[] camerasAvailable = Camera.values();

    public enum CameraMode {
        DEFAULT(1.05, 1.25),
        SIDE(1.3, 1.0);

        private final EnumMap<Camera, Double> stddev = new EnumMap<>(Camera.class);

        CameraMode(double front, double side) {
            stddev.put(Camera.FRONTLEFT, front);
            stddev.put(Camera.SIDELEFT, side);
            stddev.put(Camera.FRONTRIGHT, front);
            stddev.put(Camera.SIDERIGHT, side);
        }

        public double getStdDev(Camera camera) {
            return stddev.get(camera);
        }
    }

    public enum VisionOverride {
        NONE,
        NEXT_ONE,
        UNTIL_TARGET
    }
}