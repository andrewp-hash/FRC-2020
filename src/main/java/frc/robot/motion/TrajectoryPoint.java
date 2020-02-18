package frc.robot.motion;

import org.frcteam2910.common.math.MathUtils;
import edu.wpi.first.wpilibj.drive.Vector2d;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)
public class TrajectoryPoint {
    // This is capital Double so they get initialized to null, so the tests fail if
    // they are missing from the JSON parsing
    public Double x, y, heading, angularVelocity, time, angle;
    public Vector2d velocity;

    public static TrajectoryPoint createTrajectoryPointBetween(TrajectoryPoint before, TrajectoryPoint after,
            double percentage) {
        TrajectoryPoint newPoint = new TrajectoryPoint();
        newPoint.x = MathUtils.lerp(before.x, after.x, percentage);
        newPoint.y = MathUtils.lerp(before.y, after.y, percentage);
        newPoint.heading = MathUtils.lerp(before.heading, after.heading, percentage);
        newPoint.velocity = new Vector2d(MathUtils.lerp(before.velocity.x, after.velocity.x, percentage),
                MathUtils.lerp(before.velocity.y, after.velocity.y, percentage));
        newPoint.angularVelocity = MathUtils.lerp(before.angularVelocity, after.angularVelocity, percentage);
        newPoint.time = MathUtils.lerp(before.time, after.time, percentage);
        newPoint.angle = MathUtils.lerp(before.angle, after.angle, percentage);

        return newPoint;

    }
}