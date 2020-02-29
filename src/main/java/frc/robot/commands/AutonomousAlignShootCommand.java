package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterDistances;

public class AutonomousAlignShootCommand extends SequentialCommandGroup {

    public AutonomousAlignShootCommand(DrivetrainSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        addRequirements(shooter);

        final var spinShooterWheels = new RunCommand(() -> {
            shooter.run(ShooterDistances.FRONT_OF_TRENCH);
        }, shooter);

        final var feedBallsToShooter = new RunCommand(() -> {
            indexer.runFront();
            indexer.runBack();
        }, indexer);

        addCommands(
                parallel(new VisionAlignCommand(drive).withTimeout(3),
                        spinShooterWheels.withInterrupt(shooter::isAtSpeed)),
                parallel(spinShooterWheels, feedBallsToShooter.withTimeout(4)));
    }
}