// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/**
 * This is the Climber Routine that puts the robot from the High Bar to the
 * Traversal Bar
 * Note: This assumes the robot is on the high bar
 */
public class EndGameClimbTraverse extends SequentialCommandGroup {
  /** Creates a new EndGameClimbTraverse. */
  private ClimberSubsystem m_climberSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  public EndGameClimbTraverse(ClimberSubsystem climber, IntakeSubsystem intake, GyroSubsystem gyro) {
    m_climberSubsystem = climber;
    m_intakeSubsystem = intake;
    m_gyroSubsystem = gyro;
    addRequirements(m_climberSubsystem, m_intakeSubsystem, m_gyroSubsystem);

    addCommands(
        new TiltTo(-40, m_intakeSubsystem, m_gyroSubsystem),
        new ClimberGoTo(48, m_climberSubsystem),
        new TiltTo(-27, m_intakeSubsystem, m_gyroSubsystem),
        new ClimberGoTo(40, m_climberSubsystem),
        new ClimberGoTo(25, m_climberSubsystem),
        parallel(new ClimberGoTo(20, m_climberSubsystem), new IntakeGoTo(-15, m_intakeSubsystem)),
        new IntakeGoTo(-2, m_intakeSubsystem),
        new ClimberGoTo(0, m_climberSubsystem));
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopActuators();
    m_climberSubsystem.stop();
  }
}
