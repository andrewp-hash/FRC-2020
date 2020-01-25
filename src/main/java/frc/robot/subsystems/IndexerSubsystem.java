/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(RobotMap.INDEXER_MOTOR, MotorType.kBrushless);
  public IndexerSubsystem() {}
  public void run() {
    motor.set(0.25);
  }
  public void stop() {
    motor.set(0);
  }
}
