package frc.robot.commands;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Limelight;

public class VisionAlignCommand extends CommandBase {
    private final double kP = 0.01;
    private final double kI = 0.00;
    private final double kD = 0.00;
    private final PIDController pid = new PIDController(kP, kI, kD);
    private final DrivetrainSubsystem drive;
    private double lastTime;
    private boolean isFinished = false;

    public VisionAlignCommand(DrivetrainSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        Limelight.enableTracking();
    }

    @Override
    public void execute() {
        if (!Limelight.hasTarget())
            return;

        // from -27 to 27
        final var error = Limelight.getTargetAngle().x;

        final var rotation = pid.calculate(error, 0);

        if (error < 0.01)
            isFinished = true;

        final var now = Timer.getFPGATimestamp();
        drive.drive(Vector2.ZERO, rotation, false);
        drive.update(now, now - lastTime);
        lastTime = now;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean wasInterrupted) {
        Limelight.disableTracking();
    }
}