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
    // private final double kP = 0.000755;
    private final double kP = 0.00083;
    private final double kI = 0.0000;
    private final double kD = 0.00005;

    private final PIDController pid = new PIDController(kP, kI, kD);
    private final DrivetrainSubsystem drive;

    public VisionAlignCommand(DrivetrainSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Limelight.enableTracking();
    }

    /** Returns the vision tracking error in degrees (from -27 to 27) */
    private static double getError() {
        return Limelight.getTargetAngle().x;
    }

    @Override
    public void execute() {
        if (!Limelight.hasTarget())
            return;

        final var rotation = pid.calculate(getError(), 0);

        drive.drive(Vector2.ZERO, rotation, false);
    }

    public static boolean isAligned() {
        // If it is facing the goal and done rotating
        return getError() < 0.1 && DrivetrainSubsystem.getInstance().getAngularVelocity() < 0.5;
    }

    @Override
    public void end(boolean wasInterrupted) {
        // Limelight.disableTracking();
    }
}