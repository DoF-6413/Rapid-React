// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * This routine extends the climber in preperation for climbing
 */
public class BeginClimb extends SequentialCommandGroup {
  private ClimberSubsystem m_climberSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  /** Creates a new EndGameClimbMid. */
  public BeginClimb(ClimberSubsystem climber, IntakeSubsystem intake) {
    m_climberSubsystem = climber;
    m_intakeSubsystem = intake;
    addRequirements(m_climberSubsystem, m_intakeSubsystem);

    addCommands(
       new ActuatorUp(m_intakeSubsystem),
       new InstantCommand(() -> m_climberSubsystem.goUpManual(Constants.k_climberMaxUp)),
       new WaitCommand(Constants.oneSecond),
       new InstantCommand(() -> m_climberSubsystem.stop()),
       new WaitCommand(Constants.oneSecond),
       new ClimberGoTo(Constants.k_climberTop, Constants.k_climberMaxUp ,  m_climberSubsystem)
        );

  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
  }
}
