package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousAlignShootCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrenchAuto extends SequentialCommandGroup {

    public TrenchAuto(DrivetrainSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem) {
        addCommands(new FollowPathCommand(driveSubsystem, "Start line towards trench and shoot 3"),
                new AutonomousAlignShootCommand(driveSubsystem, shooterSubsystem, indexerSubsystem),
                new FollowPathCommand(driveSubsystem, "Grab 5 from trench"));
    }
}