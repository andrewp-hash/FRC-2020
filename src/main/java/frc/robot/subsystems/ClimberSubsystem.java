/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbMotor = new TalonFX(RobotMap.CLIMB_MOTOR);
  private final DoubleSolenoid brake = new DoubleSolenoid(RobotMap.CLIMBER_AIR_OUT, RobotMap.CLIMBER_AIR_IN);
  private boolean isRunning = false;
  public ClimberSubsystem() {
    climbMotor.configFactoryDefault(30);
    climbMotor.config_kF(1, 0, 30);
		climbMotor.config_kP(1, 0.1, 30);
		climbMotor.config_kI(1, 0, 30);
		climbMotor.config_kD(1, 0, 30);
    climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
  }
  
  @Override
  public void periodic() {
    if (isRunning) {
      climbMotor.set(ControlMode.PercentOutput, -.75);
    } else {
      climbMotor.set(ControlMode.PercentOutput, 0);
    } 
    isRunning = false;
  }
  public void run(double percent) {
    double output = 0.35 * percent;

    if (Math.abs(output) < .01) {
      output = 0;
      brake.set(Value.kForward);
    } else {
      brake.set(Value.kReverse);
    }

    climbMotor.set(ControlMode.PercentOutput, 0.35 * percent);
  }
}
