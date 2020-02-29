package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.motion.TrajectoryPoint;
import frc.robot.motion.Trajectory;

import org.frcteam2910.common.math.Vector2;

public class FollowPathCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double lastTime;
    private Trajectory trajectory;
    private double startTime = 0;

    public FollowPathCommand(DrivetrainSubsystem drivetrain, String pathName) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        trajectory = Trajectory.fromJSON(pathName);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        var timeStamp = now - startTime;
        TrajectoryPoint beforePoint = null;
        TrajectoryPoint afterPoint = null;
        TrajectoryPoint betweenPoint = null;
        var lastPoint = trajectory.points[trajectory.points.length - 1];
        if (timeStamp <= trajectory.points[0].time) {
            betweenPoint = trajectory.points[0];
        } else if (timeStamp >= lastPoint.time) {
            betweenPoint = lastPoint;
        } else {
            for (int i = 0; i < trajectory.points.length; i++) {
                var point = trajectory.points[i];
                if (timeStamp > point.time) {
                    afterPoint = point;
                    beforePoint = i > 0 ? trajectory.points[i - 1] : point;
                }
            }

            var percent = (timeStamp - beforePoint.time) / (afterPoint.time - beforePoint.time);
            betweenPoint = TrajectoryPoint.createTrajectoryPointBetween(beforePoint, afterPoint, percent);
        }

        final var kF_x = 0.006735;
        final var kF_y = 0.00684;

        drivetrain.drive(new Vector2(betweenPoint.velocity.x, betweenPoint.velocity.y).multiply(kF_x, kF_y),
                0 * betweenPoint.angularVelocity, true);
        // drivetrain.drive(new Vector2(0, 0.5).multiply(kF, kF), 0 *
        // betweenPoint.angularVelocity, true);
        drivetrain.update(now, now - lastTime);
        lastTime = now;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
