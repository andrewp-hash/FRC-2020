package frc.robot.commands.autonomous;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForward extends CommandBase {
    private final DrivetrainSubsystem m_driveSubsystem;
    private double endTime;
    private double lastTime;

    public DriveForward(DrivetrainSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + 1;
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(new Vector2(0, 0.5), 0, false);
        double time = Timer.getFPGATimestamp();
        m_driveSubsystem.update(time, time - lastTime);
        lastTime = time;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }
}