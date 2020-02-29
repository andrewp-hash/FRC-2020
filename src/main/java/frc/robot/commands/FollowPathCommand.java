package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.motion.TrajectoryPoint;
import frc.robot.motion.Trajectory;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class FollowPathCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private double lastTime;
    private Trajectory trajectory;
    private double startTime = 0;
    private double endTime = 0;
    public static boolean isFirstPath = true;

    private final double translation_kF_x = 0.006735;
    private final double translation_kF_y = 0.00684;
    private final double translation_kP = 0.0;
    private final double translation_kI = 0.0;
    private final double translation_kD = 0.0;

    private final double rotation_kF = 0.001;
    private final double rotation_kP = 0.0;
    private final double rotation_kI = 0.0;
    private final double rotation_kD = 0.0;
    private final PIDController pid_x = new PIDController(translation_kP, translation_kI, translation_kD);
    private final PIDController pid_y = new PIDController(translation_kP, translation_kI, translation_kD);
    private final PIDController pid_rotation = new PIDController(rotation_kP, rotation_kI, rotation_kD);

    public FollowPathCommand(DrivetrainSubsystem drivetrain, String pathName) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        trajectory = Trajectory.fromJSON(pathName);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        var lastPoint = trajectory.points[trajectory.points.length];
        endTime = startTime + lastPoint.time;
        if (isFirstPath) {
            var firstPoint = trajectory.points[0];
            drivetrain.resetPose(new Vector2(firstPoint.x, firstPoint.y), Rotation2.fromDegrees(firstPoint.angle));
        }

        isFirstPath = false;
    }

    @Override
    public void execute() {
        final var now = Timer.getFPGATimestamp();
        final var timeStamp = now - startTime;
        TrajectoryPoint beforePoint = null;
        TrajectoryPoint afterPoint = null;
        TrajectoryPoint betweenPoint = null;
        final var lastPoint = trajectory.points[trajectory.points.length - 1];
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

        final var currentPose = drivetrain.getScaledPose();

        final var feedForwardTranslationVector = new Vector2(betweenPoint.velocity.x, betweenPoint.velocity.y)
                .multiply(translation_kF_x, translation_kF_y);
        final var translationVector = feedForwardTranslationVector.add(
                pid_x.calculate(currentPose.translation.x, betweenPoint.x),
                pid_y.calculate(currentPose.translation.y, betweenPoint.y));

        final var rotationPidResult = pid_rotation.calculate(currentPose.rotation.toRadians(), betweenPoint.angle);
        final var rotationResult = betweenPoint.angularVelocity * rotation_kF + rotationPidResult;

        drivetrain.drive(translationVector, rotationResult, true);
        drivetrain.update(now, now - lastTime);
        lastTime = now;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }
}
