package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCommand extends CommandBase {
  private final ClimberSubsystem m_climber;
  private XboxController driverController;

  public RunClimberCommand(ClimberSubsystem climber, XboxController controller) {
    m_climber = climber;
    addRequirements(climber);
    this.driverController = controller;
  }

  @Override
  public void execute() {
    double upSpeed = driverController.getRawAxis(3);
    double downSpeed = driverController.getRawAxis(2);
    if (upSpeed > 0) {
      m_climber.run(upSpeed);
    } else {
      m_climber.run(-downSpeed);
    }
  }
}