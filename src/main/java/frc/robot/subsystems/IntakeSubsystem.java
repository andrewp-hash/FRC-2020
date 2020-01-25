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

public class IntakeSubsystem extends SubsystemBase {
 
  private final CANSparkMax intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
  private final DoubleSolenoid intakePiston = new DoubleSolenoid(0, 1);
  private boolean isRunning = false;
  
  public IntakeSubsystem() {

  }
  public void extend() {
    intakePiston.set(Value.kForward);
  }
  public void retract() {
    intakePiston.set(Value.kReverse);

  }
  public void intake() {
    intakeMotor.set(0.5);
  }
  public void outtake() {
    intakeMotor.set(-0.25);
  }
}
