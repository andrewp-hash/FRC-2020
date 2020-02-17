package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX upperMotor = new TalonFX(RobotMap.UPPER_SHOOTER_MOTOR);
  private final TalonFX lowerMotor = new TalonFX(RobotMap.LOWER_SHOOTER_MOTOR);
  private boolean isRunning = false;

  public ShooterSubsystem() {
    upperMotor.configFactoryDefault(40);
    lowerMotor.configFactoryDefault(40);
    upperMotor.config_kF(1, 0, 30);
		upperMotor.config_kP(1, 0.1, 30);
		upperMotor.config_kI(1, 0, 30);
		upperMotor.config_kD(1, 0, 30);
    upperMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
}
 public void run() {
   isRunning = true;
 }
  @Override
  public void periodic() {
    if (isRunning) {
      upperMotor.set(ControlMode.PercentOutput, -.65);
      lowerMotor.set(ControlMode.PercentOutput, .85);
      // upperMotor.set(ControlMode.Velocity, 0.75);
      // lowerMotor.set(ControlMode.Velocity, 0.75);
    } else {
      upperMotor.set(ControlMode.PercentOutput, 0);
      lowerMotor.set(ControlMode.PercentOutput, 0);
    } 
    isRunning = false;
  }
}