package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX upperMotor = new TalonFX(RobotMap.UPPER_SHOOTER_MOTOR);
  private final TalonFX lowerMotor = new TalonFX(RobotMap.LOWER_SHOOTER_MOTOR);
  private boolean isRunning = false;
  private ShooterDistances distance = ShooterDistances.BEHIND_TRENCH;
  private int lower = 0;
  private int upper = 0;
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable shooterTable = nt.getTable("/shooter");
  private final NetworkTableEntry upperErrorEntry = shooterTable.getEntry("error/upper");
  private final NetworkTableEntry lowerErrorEntry = shooterTable.getEntry("error/lower");

  public enum ShooterDistances {
    BEHIND_TRENCH, FRONT_OF_TRENCH, BEHIND_LINE
  }

  public ShooterSubsystem() {
    upperMotor.configFactoryDefault(40);
    lowerMotor.configFactoryDefault(40);
    lowerMotor.enableVoltageCompensation(true);
    upperMotor.enableVoltageCompensation(true);
    double kP = 0.5;
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
    final var lowerError = encToRPM(lowerMotor.getSelectedSensorVelocity()) - lower;
    final var upperError = encToRPM(upperMotor.getSelectedSensorVelocity()) - upper;
    upperErrorEntry.setDouble(upperError);
    lowerErrorEntry.setDouble(lowerError);
    return Math.abs(lowerError) < 50 && Math.abs(upperError) < 50;
  }

  @Override
  public void periodic() {
    lower = 0;
    upper = 0;
    if (isRunning) {
      if (distance == ShooterDistances.BEHIND_LINE) {
        upper = RobotMap.isPractice ? -0 : -3000;
        lower = RobotMap.isPractice ? 5250 : 2000;
      } else if (distance == ShooterDistances.FRONT_OF_TRENCH) {
        upper = RobotMap.isPractice ? -4875 : -2300;
        lower = RobotMap.isPractice ? 3920 : 3100;
      } else if (distance == ShooterDistances.BEHIND_TRENCH) {
        upper = RobotMap.isPractice ? -5190 : 4500;

        lower = RobotMap.isPractice ? 3920 : 3500;

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