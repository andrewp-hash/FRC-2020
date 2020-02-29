/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunIndexerCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.autonomous.TrenchAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpinnerSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterDistances;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem mDrivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem mIndexerSubsystem = new IndexerSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final SpinnerSubsystem mSpinnerSubsystem = new SpinnerSubsystem();
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final JoystickButton shooterCloseButton = new JoystickButton(operatorController, 3);
  private final JoystickButton shooterMediumButton = new JoystickButton(operatorController, 1);
  private final JoystickButton shooterFarButton = new JoystickButton(operatorController, 2);
  private final JoystickButton intakeButton = new JoystickButton(operatorController, 6);
  private final JoystickButton outtakeButton = new JoystickButton(operatorController, 5);
  private final JoystickButton spinnerButton = new JoystickButton(driverController, 6);
  private final JoystickButton visionTrackingButton = new JoystickButton(driverController, 7);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand(mDrivetrainSubsystem, driverController));
    mClimberSubsystem.setDefaultCommand(new RunClimberCommand(mClimberSubsystem, driverController));
    mIndexerSubsystem.setDefaultCommand(new RunIndexerCommand(mIndexerSubsystem, operatorController));
  }

  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // controller.get().whenPressed(new InstantCommand(() ->
    // DrivetrainSubsystem.getInstance().resetGyroAngle(Rotation2.ZERO)
    // ));
    shooterCloseButton.whileHeld(() -> mShooterSubsystem.run(ShooterDistances.BEHIND_LINE), mShooterSubsystem);
    shooterMediumButton.whileHeld(() -> mShooterSubsystem.run(ShooterDistances.FRONT_OF_TRENCH), mShooterSubsystem);
    shooterFarButton.whileHeld(() -> mShooterSubsystem.run(ShooterDistances.BEHIND_TRENCH), mShooterSubsystem);

    intakeButton.whileHeld(() -> mIntakeSubsystem.intake(), mIntakeSubsystem);
    intakeButton.whenPressed(() -> mIntakeSubsystem.extend(), mIntakeSubsystem);
    intakeButton.whenReleased(() -> mIntakeSubsystem.retract(), mIntakeSubsystem);
    intakeButton.whenReleased(() -> mIntakeSubsystem.stop(), mIntakeSubsystem);

    outtakeButton.whileHeld(() -> mIntakeSubsystem.outtake(), mIntakeSubsystem);
    outtakeButton.whenReleased(() -> mIntakeSubsystem.stop(), mIntakeSubsystem);

    spinnerButton.whenPressed(() -> mSpinnerSubsystem.run(), mSpinnerSubsystem);
    spinnerButton.whenReleased(() -> mSpinnerSubsystem.stop(), mSpinnerSubsystem);

    visionTrackingButton.whileHeld(new VisionAlignCommand(mDrivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new DriveForward(mDrivetrainSubsystem);
    // return new TestPathAuto(mDrivetrainSubsystem);
    return new TrenchAuto(mDrivetrainSubsystem, mShooterSubsystem, mIndexerSubsystem);
  }
}
