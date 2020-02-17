package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase {
  private final IntakeSubsystem m_intake;

  public StopIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
      m_intake.stop();
  }
}