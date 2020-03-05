/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.autonomous.DriveForward;
import frc.robot.commands.autonomous.Steal2Auto;
import frc.robot.commands.autonomous.TestPathAuto;
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
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final SpinnerSubsystem spinner = new SpinnerSubsystem();
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final JoystickButton shooterCloseButton = new JoystickButton(operatorController, 3);
  private final JoystickButton shooterMediumButton = new JoystickButton(operatorController, 1);
  private final JoystickButton shooterFarButton = new JoystickButton(operatorController, 2);
  private final JoystickButton intakeButton = new JoystickButton(operatorController, 6);
  private final JoystickButton outtakeButton = new JoystickButton(operatorController, 5);
  private final JoystickButton spinnerButton = new JoystickButton(driverController, 6);
  private final JoystickButton visionTrackingButton = new JoystickButton(driverController, 1);
  private final Button runIndexerButton = new AxisTrigger(operatorController, 3);
  private final Button runIndexerReverseButton = new AxisTrigger(operatorController, 2);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand(drivetrain, driverController));
    climber.setDefaultCommand(new RunClimberCommand(climber, driverController));

    autoChooser.addOption("Drive Forward Only", new DriveForward(drivetrain));
    autoChooser.addOption("Test Path (not for matches)", new TestPathAuto(drivetrain));
    autoChooser.setDefaultOption("Trench", new TrenchAuto(drivetrain, shooter, indexer, intake));
    autoChooser.addOption("Steal 2", new Steal2Auto(drivetrain, shooter, indexer, intake));

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    shooterCloseButton.whileHeld(() -> shooter.run(ShooterDistances.BEHIND_LINE), shooter);
    shooterMediumButton.whileHeld(() -> shooter.run(ShooterDistances.FRONT_OF_TRENCH), shooter);
    shooterFarButton.whileHeld(() -> shooter.run(ShooterDistances.BEHIND_TRENCH), shooter);

    intakeButton.whileHeld(intake::intake, intake);
    intakeButton.whenPressed(intake::extend, intake);
    intakeButton.whenReleased(intake::retract, intake);
    intakeButton.whenReleased(intake::stop, intake);

    outtakeButton.whileHeld(intake::outtake, intake);
    outtakeButton.whenReleased(intake::stop, intake);

    spinnerButton.whenPressed(spinner::run, spinner);
    spinnerButton.whenReleased(spinner::stop, spinner);

    visionTrackingButton.whileHeld(new VisionAlignCommand(drivetrain));

    runIndexerButton.whileHeld(indexer::feedToShooter, indexer);
    runIndexerReverseButton.whileHeld(indexer::reverse, indexer);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
