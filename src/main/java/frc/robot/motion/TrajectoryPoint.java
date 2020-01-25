package frc.robot.motion;

import java.util.Vector;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;

public class TrajectoryPoint {
    public double x, y, heading, angularVelocity, time;
    public Vector2 velocity;

    public TrajectoryPoint(double x, double y, double heading, Vector2 velocity, double angularVelocity, double time) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocity = velocity;
        this.angularVelocity = angularVelocity;
        this.time = time;
    }

    public static TrajectoryPoint createTrajectoryPointBetween(TrajectoryPoint before, TrajectoryPoint after, double percentage) {
        double x = MathUtils.lerp(before.x, after.x, percentage);
        double y = MathUtils.lerp(before.y, after.y, percentage);
        double heading = MathUtils.lerp(before.heading, after.heading, percentage);
        Vector2 velocity = new Vector2(
            MathUtils.lerp(before.velocity.x, after.velocity.x, percentage),
            MathUtils.lerp(before.velocity.y, after.velocity.y, percentage)
        );
        double angularVelocity = MathUtils.lerp(before.angularVelocity, after.angularVelocity, percentage);
        double time = MathUtils.lerp(before.time, after.time, percentage);
        return new TrajectoryPoint(x, y, heading, velocity, angularVelocity, time);
    }
}