package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends CommandBase {
    DrivetrainSubsystem drivetrain;
    XboxController controller;
    double lastTime;
    public DriveCommand(DrivetrainSubsystem drivetrain, XboxController controller) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double forward = controller.getRawAxis(1);
        double strafe = controller.getRawAxis(0);
        double rotationRaw = controller.getRawAxis(4);
        double rotation = -0.8 * Math.pow(rotationRaw, 2) * Math.signum(rotationRaw);

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
