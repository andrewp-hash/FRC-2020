package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousAlignShootCommand;
import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterDistances;

public class TrenchAuto extends SequentialCommandGroup {

        public TrenchAuto(DrivetrainSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer,
                        IntakeSubsystem intake) {
                addRequirements(shooter);

                addCommands(new FollowPathCommand(drive, "Start line towards trench and shoot 3")
                                .deadlineWith(new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE))),
                                new AutonomousAlignShootCommand(drive, shooter, indexer),
                                new InstantCommand(intake::extend),
                                new WaitCommand(0.5).deadlineWith(new VisionAlignCommand(drive)),
                                new FollowPathCommand(drive, "Aim to 3 from trench")
                                                .deadlineWith(new RunCommand(intake::intake)),
                                new InstantCommand(intake::retract), new InstantCommand(intake::stop),
                                new FollowPathCommand(drive, "3 from trench to aim").deadlineWith(
                                                new RunCommand(() -> shooter.run(ShooterDistances.BEHIND_LINE))),
                                new AutonomousAlignShootCommand(drive, shooter, indexer));
        }

}