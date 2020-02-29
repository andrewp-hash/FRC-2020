package frc.robot;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry camMode = table.getEntry("camMode");
    private static NetworkTableEntry ledMode = table.getEntry("ledMode");

    public static void enableTracking() {
        camMode.setNumber(0);
        ledMode.setNumber(3);
    }

    public static void disableTracking() {
        camMode.setNumber(1);
        ledMode.setNumber(1);
    }

    public static boolean hasTarget() {
        return table.getEntry("tv").getBoolean(true);
    }

    public static Vector2 getTargetAngle() {
        return new Vector2(table.getEntry("tx").getNumber(0).doubleValue(),
                table.getEntry("ty").getNumber(0).doubleValue());
    }
}