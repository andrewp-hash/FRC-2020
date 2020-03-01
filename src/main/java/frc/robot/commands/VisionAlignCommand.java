package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Limelight;

public class VisionAlignCommand extends CommandBase {
    // option 1 - just p
    // private final double kP = 0.000625;
    // private final double kI = 0.0000;
    // private final double kD = 0.00;

    // option 2 - pi
    // private final double kP = 0.000425;
    // private final double kI = 0.0008;
    // private final double kD = 0.00;

    // option 3 - pd
    private final double kP = 0.000755;
    private final double kI = 0.0000;
    private final double kD = 0.00005;

    private final PIDController pid = new PIDController(kP, kI, kD);
    private final DrivetrainSubsystem drive;
    private boolean isFinished = false;

    public VisionAlignCommand(DrivetrainSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Limelight.enableTracking();
    }

    @Override
    public void execute() {
        if (!Limelight.hasTarget())
            return;

        // from -27 to 27
        final var error = Limelight.getTargetAngle().x;

        final var rotation = pid.calculate(error, 0);

        // If it is facing the goal and done rotating
        if (error < 0.05 && drive.getAngularVelocity() < 0.5)
            isFinished = true;

        drive.drive(Vector2.ZERO, rotation, false);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean wasInterrupted) {
        // Limelight.disableTracking();
    }
}