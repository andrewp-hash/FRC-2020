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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.buttons.Button;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSubsystem mDrivetrainSubsystem = DrivetrainSubsystem.getInstance();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem mIndexerSubsystem = new IndexerSubsystem();
  private XboxController controller = new XboxController(0);
  private JoystickButton shooterButton = new JoystickButton(controller, 1);
  private JoystickButton intakeButton = new JoystickButton(controller, 6);
  private JoystickButton outtakeButton = new JoystickButton(controller, 5);
  private JoystickButton indexerButton = new JoystickButton(controller, 3);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand(mDrivetrainSubsystem));
    mIndexerSubsystem.setDefaultCommand(new InstantCommand(() -> mIndexerSubsystem.stop()));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // controller.get().whenPressed(new InstantCommand(() ->
    //   DrivetrainSubsystem.getInstance().resetGyroAngle(Rotation2.ZERO)
    // ));
    shooterButton.whileHeld(new InstantCommand(() ->
      mShooterSubsystem.run()
    ));
    
    intakeButton.whileHeld(new InstantCommand(() ->
      mIntakeSubsystem.intake()
    ));
    intakeButton.whenPressed(new InstantCommand(() ->
      mIntakeSubsystem.extend()
    ));

    outtakeButton.whileHeld(new InstantCommand(() ->
      mIntakeSubsystem.outtake()
    ));
    outtakeButton.whenPressed(new InstantCommand(() ->
      mIntakeSubsystem.retract()
    ));

    indexerButton.whileHeld(new InstantCommand(() ->
      mIndexerSubsystem.run()
    ));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
