// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final AutoCommand m_autoCommand = new
  // AutoCommand(m_drivetrainSubsystem);
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final AutoCommand m_autoCommand = new AutoCommand(m_drivetrainSubsystem, m_shooterSubsystem);
  public Joystick m_leftStick = new Joystick(Constants.initialJoystickPort);
  public Joystick m_rightStick = new Joystick (Constants.secondaryJoystickPort);
  public XboxController m_xbox = new XboxController (Constants.xboxPort);
  

  public Joystick LeftStick;
  public Joystick RightStick;

  // XBOX Contoller Defs (For intake and Climber)

  // Intake Subsystem
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    // Sets deafault Drivetrain for Subsystem
    m_drivetrainSubsystem.setDefaultCommand(
        new RunCommand(() -> m_drivetrainSubsystem.setRaw(m_leftStick.getRawAxis(Constants.joystickXAxis),
            m_rightStick.getRawAxis(Constants.joystickYAxis)), m_drivetrainSubsystem));

    // Trigger ButtonA = Spins Intake and Indexer forwards (Towards Shooter)
    new JoystickButton(m_xbox, XboxController.Button.kA.value)
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.spinMotor(), m_intakeSubsystem))
        .whenPressed(new InstantCommand(() -> m_indexerSubsystem.spinMotor(), m_indexerSubsystem))
        .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
        .whenReleased(new RunCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));
    // Trigger ButtonB = Spins Intake and Indexer backwards (Away from Shooter)
    new JoystickButton(m_xbox, XboxController.Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_intakeSubsystem.reverseMotor(), m_intakeSubsystem))
        .whenPressed(new InstantCommand(() -> m_indexerSubsystem.spinBack(), m_indexerSubsystem))
        .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
        .whenReleased(new RunCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));
    // Trigger ButtonX = Brings Actuator Up and Will Stop When Released or When
    // Limit Switch Get Hits
    new JoystickButton(m_xbox, XboxController.Button.kX.value)
        .whenPressed(new RunCommand(() -> m_intakeSubsystem.setActuatorUp(Constants.slowSpeed), m_intakeSubsystem))
        .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopActuators(), m_intakeSubsystem));
    // Trigger ButtonY = Bring Actuator Down and Will Stop When Released or When
    // Limit Switches Get Hits
    new JoystickButton(m_xbox, XboxController.Button.kY.value)
        .whenPressed(new RunCommand(() -> m_intakeSubsystem.setActuatorDown(Constants.slowSpeed), m_intakeSubsystem))
        .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopActuators(), m_intakeSubsystem));
    // Trigger Button Left Bumper (L1) = Runs Shooter Subsystem at 2000 R P M
    new JoystickButton(m_xbox, XboxController.Button.kLeftBumper.value)
        .whenPressed(new InstantCommand(() -> m_shooterSubsystem.enable(), m_shooterSubsystem))
        .whenPressed(new InstantCommand(() -> m_shooterSubsystem.LowerHub(), m_shooterSubsystem))
        .whenReleased((new InstantCommand(() -> m_shooterSubsystem.disable(), m_shooterSubsystem)));
    // Trigger Button Right Bumper (R1) = Runs Shooter Subsystem at 5000 R P M
    new JoystickButton(m_xbox, XboxController.Button.kRightBumper.value)
        .whenPressed(new InstantCommand(() -> m_shooterSubsystem.enable(), m_shooterSubsystem))
        .whenPressed(new InstantCommand(() -> m_shooterSubsystem.UpperHub(), m_shooterSubsystem))
        .whenReleased((new InstantCommand(() -> m_shooterSubsystem.disable(), m_shooterSubsystem)));

    SmartDashboard.putData(m_shooterSubsystem);
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
    return m_autoCommand;
  }
}
