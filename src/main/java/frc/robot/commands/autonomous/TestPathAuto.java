package frc.robot.commands.autonomous;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestPathAuto extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_driveSubsystem;

    public TestPathAuto(DrivetrainSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
        addCommands(new FollowPathCommand(m_driveSubsystem, "Measurement"));
    }
}