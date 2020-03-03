package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterDistances;

/**
 * In parallel spin up the shooter wheels and vision align (timeout 1.5). Once
 * both are done, feeds balls into shooter for 3 seconds
 */
public class AutonomousAlignShootCommand extends ParallelCommandGroup {
    public AutonomousAlignShootCommand(DrivetrainSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer) {

        addCommands(sequence(new WaitCommand(1.5), new WaitUntilCommand(shooter::isAtSpeed),
                new RunCommand(indexer::feedToShooter).withTimeout(2)).deadlineWith(
                        new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE)),
                        new VisionAlignCommand(drive)));
    }
}
