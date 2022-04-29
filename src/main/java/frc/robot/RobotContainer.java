// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.*;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // private final AutoCommand m_autoCommand = new
  // AutoCommand(m_drivetrainSubsystem);
  public static IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  public static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
  public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  
  // public Joystick m_leftStick = new Joystick(Constants.initialJoystickPort);
  // public Joystick m_rightStick = new Joystick(Constants.secondaryJoystickPort);
  public static XboxController m_driverXbox = new XboxController(0);
  // not used
  //public static XboxController m_xbox = new XboxController(1);
  public Trigger driverLeftTrigger = new Trigger(() -> IntakeSubsystem.getLeftTriggerActive());
  public Trigger driverRightTrigger = new Trigger(() -> IntakeSubsystem.getRightTriggerActive());
  // XBOX Contoller Defs (For intake and Climber)

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(
   ) {
     

    // Configure the button bindings
    configureButtonBindings();
    // Sets deafault Drivetrain for Subsystem
    m_drivetrainSubsystem.setDefaultCommand(
        new RunCommand(() -> m_drivetrainSubsystem.setRaw(m_driverXbox.getLeftY(), 
        m_driverXbox.getRightX()), m_drivetrainSubsystem));
//Left Trigger on Joystick = Make the climber go up



driverRightTrigger
.whenActive(new InstantCommand(() -> m_intakeSubsystem.spinMotor(), m_intakeSubsystem))
.whenActive(new InstantCommand(() -> m_indexerSubsystem.spinMotor(), m_indexerSubsystem))
.whenInactive(new InstantCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
.whenInactive(new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));

  driverLeftTrigger
  .whenActive(new InstantCommand(() -> m_intakeSubsystem.reverseMotor(), m_intakeSubsystem))
  .whenActive(new InstantCommand(() -> m_indexerSubsystem.spinBack(), m_indexerSubsystem))
  .whenInactive(new InstantCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
  .whenInactive(new  InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));


    // new JoystickButton(m_xbox, XboxController.Button.kStart.value)
    // .whenPressed(new intakeGoTo(-20));


    // Trigger Button Left Bumper (L1) = Runs Shooter Subsystem at 2000 R P M
    new JoystickButton(m_driverXbox, XboxController.Button.kLeftBumper.value)
    .whenHeld(new ShootTeleopLow(m_shooterSubsystem, m_indexerSubsystem))
    .whenReleased((new InstantCommand(() -> m_shooterSubsystem.disable(), m_shooterSubsystem)))
    .whenReleased((new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem)));
  
  
    // Trigger Button Right Bumper (R1) = Runs Shooter Subsystem at 5000 R P M

    new JoystickButton(m_driverXbox, XboxController.Button.kRightBumper.value)
    .whenHeld(new ShootTeleopHigh(m_shooterSubsystem, m_indexerSubsystem))
    .whenReleased((new InstantCommand(() -> m_shooterSubsystem.disable(), m_shooterSubsystem)))
    .whenReleased((new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem)));

  // leftTrigger
  // .whenActive(new EndGameClimbMid());

  
    // new JoystickButton(m_rightStick, 7).whenPressed(new InstantCommand(() -> m_gyroSubsystem.resetYaw(), m_drivetrainSubsystem));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings()

  {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // new AutoCommand(DrivetrainSubsystem, 5);
    return null;
  }
}
