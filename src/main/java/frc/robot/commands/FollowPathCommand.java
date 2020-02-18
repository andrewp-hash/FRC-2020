package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Robot;
import frc.robot.motion.TrajectoryPoint;

import org.frcteam2910.common.math.Vector2;

public class FollowPathCommand extends CommandBase {
    DrivetrainSubsystem drivetrain;
    double lastTime;

    public FollowPathCommand(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        // TrajectoryPoint beforePoint = new TrajectoryPoint();
        // TrajectoryPoint afterPoint = new TrajectoryPoint();

        // TrajectoryPoint betweenPoint =
        // TrajectoryPoint.createTrajectoryPointBetween(beforePoint, afterPoint, 0.5);

        // drivetrain.drive(new Vector2(forward, strafe), rotation, true);
        double time = Timer.getFPGATimestamp();
        drivetrain.update(time, time - lastTime);
        lastTime = time;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
