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
  private ShooterDistances distance = ShooterDistances.BEHIND_TRENCH;

  public enum ShooterDistances {
    BEHIND_TRENCH, FRONT_OF_TRENCH, BEHIND_LINE
  }

  public ShooterSubsystem() {
    upperMotor.configFactoryDefault(40);
    lowerMotor.configFactoryDefault(40);
    upperMotor.config_kF(1, 0, 30);
    upperMotor.config_kP(1, 0.1, 30);
    upperMotor.config_kI(1, 0, 30);
    upperMotor.config_kD(1, 0, 30);
    upperMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 50);
  }

  public void stop() {
    isRunning = false;
  }

  public void run(ShooterDistances distance) {
    isRunning = true;
    this.distance = distance;
  }

  @Override
  public void periodic() {
    System.out.println(upperMotor.getSelectedSensorVelocity() / 100 * 1000 / 2048 * 60);
    if (isRunning) {
      if (distance == ShooterDistances.BEHIND_LINE) {
        upperMotor.set(ControlMode.PercentOutput, -.4);
        lowerMotor.set(ControlMode.PercentOutput, .5);
      } else if (distance == ShooterDistances.FRONT_OF_TRENCH) {
        upperMotor.set(ControlMode.PercentOutput, -.4);
        lowerMotor.set(ControlMode.PercentOutput, .27);
      } else if (distance == ShooterDistances.BEHIND_TRENCH) {
        upperMotor.set(ControlMode.PercentOutput, -.85);
        lowerMotor.set(ControlMode.PercentOutput, .35);
      }
    } else {
      upperMotor.set(ControlMode.PercentOutput, 0);
      lowerMotor.set(ControlMode.PercentOutput, 0);
    }
    isRunning = false;
  }

}