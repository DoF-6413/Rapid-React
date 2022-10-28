// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
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
    public static IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    public static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    public static GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
    public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public static LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();

    // XBOX Contoller Defs (For intake and Climber)
    public static XboxController m_driverXbox = new XboxController(Constants.driverXboxPort);
    public static XboxController m_auxXbox = new XboxController(Constants.auxXboxPort);
    public Trigger driverLeftTrigger = new Trigger(() -> IntakeSubsystem.getLeftTriggerActive());
    public Trigger driverRightTrigger = new Trigger(() -> IntakeSubsystem.getRightTriggerActive());
    public Trigger auxLeftTrigger = new Trigger(() -> ClimberSubsystem.getLeftTriggerActive());
    public Trigger auxRightTrigger = new Trigger(() -> ClimberSubsystem.getRightTriggerActive());

    // Different Autos
    private final Command m_autoHighIntake = new AutoHighIntake(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_wAutoLowIntake = new WAutoLowIntake(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_autoHighGoal = new AutoHighGoal(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_autoLowGoal = new AutoLowGoal(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_autoMove = new AutoMove(m_drivetrainSubsystem, m_intakeSubsystem);

    private final Command m_autoPush = new AutoPush(m_drivetrainSubsystem, m_intakeSubsystem);

    private final Command m_wHighIntake = new WAutoHighIntake(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_autoLowIntake = new AutoLowIntake(m_drivetrainSubsystem, m_shooterSubsystem,
            m_indexerSubsystem, m_intakeSubsystem);

    private final Command m_threeBallCommand = new ThreeBallAuto(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem, m_gyroSubsystem);
    // Intake Subsystem
    public SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        
        /********************************************** * Pilot Controls Below********************************************************** */
        
        // Sets default Drivetrain
        m_drivetrainSubsystem.setDefaultCommand(
                new RunCommand(() -> m_drivetrainSubsystem.setRaw(m_driverXbox.getLeftY(),
                        m_driverXbox.getRightX()), m_drivetrainSubsystem));

        //Runs Intake In
        driverRightTrigger
            .whenActive(new InstantCommand(() -> m_intakeSubsystem.spinMotor(), m_intakeSubsystem))
            .whenActive(new InstantCommand(() -> m_indexerSubsystem.spinMotor(), m_indexerSubsystem))
            .whenInactive(new InstantCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
            .whenInactive(new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));
                
        //Runs Intake Out
        driverLeftTrigger
            .whenActive(new InstantCommand(() -> m_intakeSubsystem.reverseMotor(), m_intakeSubsystem))
            .whenActive(new InstantCommand(() -> m_indexerSubsystem.spinBack(), m_indexerSubsystem))
            .whenInactive(new InstantCommand(() -> m_intakeSubsystem.stopMotor(), m_intakeSubsystem))
            .whenInactive(new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem));
                
        /********************************************** *Aux Controls Below********************************************************** */
        
        // Button Nearest XYAB Buttons
        // Begin Climb Code
        new JoystickButton(m_auxXbox, XboxController.Button.kStart.value)
        .whenPressed(new BeginClimb(m_climberSubsystem, m_intakeSubsystem));
        
        // Button Nearest D-Pad
        // End Climb Code
        new JoystickButton(m_auxXbox, XboxController.Button.kBack.value)
        .whenPressed(new EndClimb(m_climberSubsystem));
        
        // Button A = Climber Up
        new JoystickButton(m_auxXbox, XboxController.Button.kA.value)
        .whenPressed
        (
            new RunCommand(() -> m_climberSubsystem.goUpManual(Constants.k_climberMaxUp),
        m_climberSubsystem))
        .whenReleased(new RunCommand(() -> m_climberSubsystem.stop(), m_climberSubsystem));
            // new ClimberGoTo(Constants.k_climberTop, Constants.k_climberMaxUp, m_climberSubsystem));
        
        // Buttom B = Climber Down
        new JoystickButton(m_auxXbox, XboxController.Button.kB.value)
        .whenPressed(
            new RunCommand(() -> m_climberSubsystem.goDownManual(Constants.k_climberMaxDown),
            m_climberSubsystem))
            .whenReleased(new RunCommand(() -> m_climberSubsystem.stop(), m_climberSubsystem));
        
        // Button Y = Brings Intake Up
        new JoystickButton(m_auxXbox, XboxController.Button.kY.value)
        .whenPressed(
            new RunCommand(() -> m_intakeSubsystem.setAllActuatorsUp(Constants.actuatorsSpeed),
            m_intakeSubsystem))
            .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopActuators(), m_intakeSubsystem));
           
        // Button X = Brings Intake Down
        new JoystickButton(m_auxXbox, XboxController.Button.kX.value)
        .whenPressed(
            new RunCommand(() -> m_intakeSubsystem.setAllActuatorsDown(Constants.actuatorsSpeed),
            m_intakeSubsystem))
            .whenReleased(new RunCommand(() -> m_intakeSubsystem.stopActuators(), m_intakeSubsystem));
        
        // Left Bumper (L1) = Not Used
        new JoystickButton(m_auxXbox, XboxController.Button.kLeftBumper.value)
        .whenPressed(
            new RunCommand(() -> m_climberSubsystem.runStingerMotor(),
            m_climberSubsystem))
            .whenReleased(new RunCommand(() -> m_climberSubsystem.stopStingerMotor(), m_climberSubsystem));

        // Right Bumper (R1) = Runs Limelight Shoot
        
        new JoystickButton(m_auxXbox, XboxController.Button.kRightBumper.value)
        .whenHeld(
            new LLAimAndShoot(m_LimelightSubsystem, m_indexerSubsystem, m_shooterSubsystem,
                                m_drivetrainSubsystem))
                                .whenReleased((new InstantCommand(() -> m_shooterSubsystem.disable(), m_shooterSubsystem)))
                                .whenReleased((new InstantCommand(() -> m_indexerSubsystem.stopMotor(), m_indexerSubsystem)));
                                
        // Right Trigger = Manual Shoot High
        auxRightTrigger
        .whenActive(new ShootTeleopHigh(m_shooterSubsystem, m_indexerSubsystem));
        
                                
        // Left Trigger = Manual Shoot Low
        auxLeftTrigger
            .whenActive( new ShootTeleopLow(m_shooterSubsystem, m_indexerSubsystem));
           
        
        SmartDashboard.putData(m_shooterSubsystem);
        m_chooser.setDefaultOption("Mid High Intake", m_autoHighIntake);
        m_chooser.addOption("Mid Low Intake", m_autoLowIntake);
        m_chooser.addOption("Wall Low Intake", m_wAutoLowIntake);
        m_chooser.addOption("Wall High Intake", m_wHighIntake);
        m_chooser.addOption("High Goal", m_autoHighGoal);
        m_chooser.addOption("Low Goal", m_autoLowGoal);
        m_chooser.addOption("Just Move", m_autoMove);
        m_chooser.addOption("Push Away", m_autoPush);
        m_chooser.addOption("Three Ball Auto", m_threeBallCommand);
        SmartDashboard.putData(m_chooser);
        m_LimelightSubsystem.getTx();
        m_LimelightSubsystem.getTy();
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
        return m_chooser.getSelected();
    }
}
