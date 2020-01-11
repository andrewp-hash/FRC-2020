package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Robot;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends CommandBase {
    DrivetrainSubsystem drivetrain;
    double lastTime;
    public DriveCommand(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        double forward = Robot.getOi().getDriveForwardAxis().get(true);
        double strafe = Robot.getOi().getDriveStrafeAxis().get(true);
        double rotation = Robot.getOi().getDriveRotationAxis().get(true);

        drivetrain.drive(new Vector2(forward, strafe), rotation, true);
        double time = Timer.getFPGATimestamp();
        drivetrain.update(time, time - lastTime);
        lastTime = time;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
