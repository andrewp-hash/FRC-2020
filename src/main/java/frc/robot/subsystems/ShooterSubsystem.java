package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX upperMotor = new TalonFX(RobotMap.UPPER_SHOOTER_MOTOR);
  private final TalonFX lowerMotor = new TalonFX(RobotMap.LOWER_SHOOTER_MOTOR);
  private boolean isRunning = false;
  private ShooterDistances distance = ShooterDistances.BEHIND_TRENCH;
  private int lower = 0;
  private int upper = 0;

  public enum ShooterDistances {
    BEHIND_TRENCH, FRONT_OF_TRENCH, BEHIND_LINE
  }

  public ShooterSubsystem() {
    upperMotor.configFactoryDefault(40);
    lowerMotor.configFactoryDefault(40);
    lowerMotor.enableVoltageCompensation(true);
    upperMotor.enableVoltageCompensation(true);
    double kP = .5;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    upperMotor.config_kF(0, kF, 30);
    upperMotor.config_kP(0, kP, 30);
    upperMotor.config_kI(0, kI, 30);
    upperMotor.config_kD(0, kD, 30);
    lowerMotor.config_kF(0, kF, 30);
    lowerMotor.config_kP(0, kP, 30);
    lowerMotor.config_kI(0, kI, 30);
    lowerMotor.config_kD(0, kD, 30);
    upperMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
    lowerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);
  }

  public void stop() {
    isRunning = false;
  }

  public void run(ShooterDistances distance) {
    isRunning = true;
    this.distance = distance;
  }

  public static double encToRPM(double enc) {
    return enc / 100 * 1000 / 2048 * 60;
  }

  public static double RPMToEnc(double rpm) {
    return rpm * 100 / 1000 * 2048 / 60;
  }

  public boolean isAtSpeed() {
    return Math.abs(encToRPM(lowerMotor.getSelectedSensorVelocity()) - lower) < 5
        && Math.abs(encToRPM(upperMotor.getSelectedSensorVelocity()) - upper) < 5;
  }

  @Override
  public void periodic() {
    lower = 0;
    upper = 0;
    // System.out.println(encToRPM(lowerMotor.getSelectedSensorVelocity(1)));
    if (isRunning) {
      if (distance == ShooterDistances.BEHIND_LINE) {
        upper = -0;
        lower = 5250;
      } else if (distance == ShooterDistances.FRONT_OF_TRENCH) {
        upper = -4875;
        lower = 3920;
      } else if (distance == ShooterDistances.BEHIND_TRENCH) {
        upper = -5190;
        lower = 3920;
      }

      if (upper == 0) {
        upperMotor.set(ControlMode.PercentOutput, 0);
      }
      lowerMotor.set(ControlMode.Velocity, RPMToEnc(lower), DemandType.ArbitraryFeedForward, lower * (.65 / 3920));
      upperMotor.set(ControlMode.Velocity, RPMToEnc(upper), DemandType.ArbitraryFeedForward, upper * (.65 / 3920));
    } else {
      upperMotor.set(ControlMode.PercentOutput, 0);
      lowerMotor.set(ControlMode.PercentOutput, 0);
    }
    isRunning = false;
  }

}