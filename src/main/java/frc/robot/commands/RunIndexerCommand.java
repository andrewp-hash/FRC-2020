package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexerCommand extends CommandBase {
  private final IndexerSubsystem m_indexer;
  private XboxController operatorController;
  private XboxController driverController;

  public RunIndexerCommand(IndexerSubsystem indexer, XboxController operatorController, XboxController driverController) {
    m_indexer = indexer;
    addRequirements(indexer);
    this.operatorController = operatorController;
    this.driverController = driverController;
  }

  @Override
  public void execute() {
    double TRIGGER_THRESHOLD = 0.75;
    if (operatorController.getRawAxis(3) > TRIGGER_THRESHOLD ||  driverController.getRawAxis(3) > TRIGGER_THRESHOLD) {
        m_indexer.runFront();
    } else {
        m_indexer.stopFront();
    }
    if (driverController.getRawAxis(3) > TRIGGER_THRESHOLD) {
        m_indexer.runBack();
    } else {
        m_indexer.stopBack();
    }
  }
}