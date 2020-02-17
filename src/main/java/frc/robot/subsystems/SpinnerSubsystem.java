/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class SpinnerSubsystem extends SubsystemBase {
 
  private final CANSparkMax spinnerMotor = new CANSparkMax(RobotMap.SPINNER_MOTOR, MotorType.kBrushless);
  private final DoubleSolenoid spinnerPiston = new DoubleSolenoid(RobotMap.SPINNER_AIR_IN, RobotMap.SPINNER_AIR_OUT);
  
  public SpinnerSubsystem() {

  }

  public void run() {
    spinnerMotor.set(0.5);
    spinnerPiston.set(Value.kForward);
  }
 
  public void stop() {
    spinnerMotor.set(0);
    spinnerPiston.set(Value.kReverse);
  }
}
