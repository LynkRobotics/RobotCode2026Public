package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Field {
    // TODO Consider using fieldLayout.getFieldLength(), etc.
    public static final Distance width = Units.Meters.of(FlippingUtil.fieldSizeY); // Units.inchesToMeters(26*12 + 5);
    public static final Distance length = Units.Meters.of(FlippingUtil.fieldSizeX); // Units.inchesToMeters(57*12 + 6.875);

    public static final Distance deepPass = Units.Meters.of(0.7);
    public static final Distance shallowPass = Units.Meters.of(3.2);
    public static final Distance nearWall = Units.Meters.of(1.0);
    public static final Distance nearMiddle = Units.Meters.of(2.9);

    public static final double zoneHysteresis = 0.25; // meters
    public static final double trenchRunHalfLength = 2.5; // meters
    public static final double trenchHysteresis = 0.25; // meters

    public enum PassTarget {
        CORNER(deepPass, nearWall),
        TRENCH(shallowPass, nearWall),
        MIDDLE(shallowPass, nearMiddle);

        public final Translation2d position;

        private PassTarget(Distance x, Distance y) {
            this.position = new Translation2d(x, y);
        }
    };

    public enum FieldSide {
        DEPOT,
        OUTPOST;
    }

    public enum Zone {
        ALLIANCE,
        TRANSITION,
        REMOTE
    }

    public enum TrenchStatus {
        IN_RUN,
        OUT_RUN
    }
}