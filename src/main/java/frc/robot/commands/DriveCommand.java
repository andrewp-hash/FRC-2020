package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private XboxController controller;

    public DriveCommand(DrivetrainSubsystem drivetrain, XboxController controller) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double forward = -controller.getRawAxis(1);
        double strafe = controller.getRawAxis(0);
        double rotationRaw = controller.getRawAxis(4);
        double rotation = -0.5 * Math.pow(rotationRaw, 2) * Math.signum(rotationRaw);

        if (Math.abs(forward) < .015) {
            forward = 0;
        }

        if (Math.abs(strafe) < .015) {
            strafe = 0;
        }

        if (Math.abs(rotation) < .015) {
            rotation = 0;
        }

        drivetrain.drive(new Vector2(strafe, forward), rotation, true);
        // drivetrain.drive(new Vector2(0, 0.1), 0, false);
        // drivetrain.drive(new Vector2(0, 0), 0.05, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
