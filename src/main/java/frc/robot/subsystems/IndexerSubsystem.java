/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax upperMotor = new CANSparkMax(RobotMap.INDEXER_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax lowerMotor = new CANSparkMax(RobotMap.INDEXER_BACK_MOTOR, MotorType.kBrushless);
  private final AnalogInput ballSensorUpper = new AnalogInput(RobotMap.BALL_SENSOR_UPPER);
  private final AnalogInput ballSensorLower = new AnalogInput(RobotMap.BALL_SENSOR_LOWER);

  public IndexerSubsystem() {
  }

  public void runBack() {
    upperMotor.set(-.15);
  }

  public void runFront() {
    lowerMotor.set(-0.55);
  }

  public void stopFront() {
    lowerMotor.set(0);
  }

  public void stopBack() {
    upperMotor.set(0);
  }

  public boolean isUpperTriggered() {
    return ballSensorUpper.getVoltage() > 1.5;
  }

  public boolean isLowerTriggered() {
    return ballSensorLower.getVoltage() > 1.5;
  }
}
