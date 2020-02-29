package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestPathAuto extends SequentialCommandGroup {

    public TestPathAuto(DrivetrainSubsystem driveSubsystem) {
        addCommands(new FollowPathCommand(driveSubsystem, "Measurement"));
    }
}