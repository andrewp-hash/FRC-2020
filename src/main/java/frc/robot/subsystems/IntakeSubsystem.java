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
  private final DoubleSolenoid intakeLeftPiston = new DoubleSolenoid(RobotMap.INTAKE_LEFT_AIR_IN,
      RobotMap.INTAKE_LEFT_AIR_OUT);
  private final DoubleSolenoid intakeRightPiston = new DoubleSolenoid(RobotMap.INTAKE_RIGHT_AIR_IN,
      RobotMap.INTAKE_RIGHT_AIR_OUT);

  public IntakeSubsystem() {

  }

  public void extend() {
    intakeRightPiston.set(Value.kReverse);
    intakeLeftPiston.set(Value.kReverse);
  }

  public void retract() {
    intakeRightPiston.set(Value.kForward);
    intakeLeftPiston.set(Value.kForward);

  }

  public void intake() {
    intakeMotor.set(-.4);
  }

  public void outtake() {
    intakeMotor.set(0.4);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}
