package frc.robot.motion;

import java.io.IOException;
import java.nio.file.Paths;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.RobotMap;

public class TrajectoryTest {
    static void assertTrajectoryPoint(TrajectoryPoint p) {
        Assertions.assertNotNull(p.x);
        Assertions.assertNotNull(p.y);
        Assertions.assertNotNull(p.angle);
        Assertions.assertNotNull(p.angularVelocity);
        Assertions.assertNotNull(p.heading);
        Assertions.assertNotNull(p.time);
        Assertions.assertNotNull(p.velocity);
        Assertions.assertNotNull(p.velocity.x);
        Assertions.assertNotNull(p.velocity.y);
    }

    @Test
    public void ReadTrajectoryTest() throws IOException {
        TrajectoryReader.loadTrajectories(Paths.get("./src/main/deploy/trajectories.json"));
        var t = Trajectory.fromJSON("Aim to 3 from trench");
        for (TrajectoryPoint p : t.points) {
            assertTrajectoryPoint(p);
        }
        t = Trajectory.fromJSON("Start line towards goal and shoot 3");
        for (TrajectoryPoint p : t.points) {
            assertTrajectoryPoint(p);
        }
    }

    @Test
    public void TrajectoryPointBetweenTest() {
        var startPoint = new TrajectoryPoint();

        startPoint.x = 0.0;
        startPoint.y = 0.1;
        startPoint.heading = 2.0;
        startPoint.angularVelocity = 1.0;
        startPoint.time = 1.0;
        startPoint.angle = 2.0;
        startPoint.velocity = new Vector2d(-10, -10);

        var endPoint = new TrajectoryPoint();

        endPoint.x = 1.0;
        endPoint.y = 0.3;
        endPoint.heading = 5.0;
        endPoint.angularVelocity = 1.0;
        endPoint.time = -2.0;
        endPoint.angle = 2.1;
        endPoint.velocity = new Vector2d(10, 12);

        var betweenPoint = TrajectoryPoint.createTrajectoryPointBetween(startPoint, endPoint, 0.5);

        Assertions.assertEquals(betweenPoint.x, 0.5);
        Assertions.assertEquals(betweenPoint.y, 0.2);
        Assertions.assertEquals(betweenPoint.heading, 3.5);
        Assertions.assertEquals(betweenPoint.angularVelocity, 1.0);
        Assertions.assertEquals(betweenPoint.time, -0.5);
        Assertions.assertEquals(betweenPoint.angle, 2.05);
        Assertions.assertEquals(betweenPoint.velocity.x, 0);
        Assertions.assertEquals(betweenPoint.velocity.y, 1);
    }
}