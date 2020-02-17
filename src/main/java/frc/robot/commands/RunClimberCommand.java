package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends CommandBase {
  private final ClimberSubsystem m_climber;
  private XboxController operatorController;

  public RunClimberCommand(ClimberSubsystem climber, XboxController controller) {
    m_climber = climber;
    addRequirements(climber);
    this.operatorController = controller;
  }

  @Override
  public void execute() {
    m_climber.run(operatorController.getRawAxis(5));
  }
}