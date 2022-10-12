// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Climb routine that brings robot from the ground onto mid bar.
 * Note: This command requires the intake to be all the way up, the climber
 * above the mid bar.
 */
public class EndGameClimbMid extends SequentialCommandGroup {
  private ClimberSubsystem m_climberSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new EndGameClimbMid. */
  public EndGameClimbMid(ClimberSubsystem climber, IntakeSubsystem intake) {
    m_climberSubsystem = climber;
    m_intakeSubsystem = intake;
    addRequirements(m_climberSubsystem, m_intakeSubsystem);

    addCommands(
        new ClimberGoTo(0, m_climberSubsystem),
        new IntakeGoTo(-2, m_intakeSubsystem),
        new WaitCommand(0.5),
        new ClimberGoTo(8, m_climberSubsystem));
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopActuators();
    m_climberSubsystem.stop();
  }
}
