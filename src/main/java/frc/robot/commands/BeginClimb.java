// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This routine extends the climber in preperation for climbing
 */
public class BeginClimb extends SequentialCommandGroup {
  private ClimberSubsystem m_climberSubsystem;

  /** Creates a new EndGameClimbMid. */
  public BeginClimb(ClimberSubsystem climber) {
    m_climberSubsystem = climber;
    addRequirements(m_climberSubsystem);

    addCommands(
        // new ClimberGoTo(0, m_climberSubsystem),
        // new IntakeGoTo(-2, m_intakeSubsystem),
        // new WaitCommand(0.5),
        // new ClimberGoTo(8, m_climberSubsystem)
        );

  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
  }
}
