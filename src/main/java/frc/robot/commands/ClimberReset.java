// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/** This resets the climber to the 0 position and makes the encoder value 0 */
public class ClimberReset extends CommandBase {
  /** Creates a new ClimberReset. */
  private ClimberSubsystem m_climberSubsystem;

  public ClimberReset(ClimberSubsystem climber) {
    m_climberSubsystem = climber;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.setCurrentLimit(20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.goDownManual(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setPosition();
    m_climberSubsystem.stop();
    m_climberSubsystem.setCurrentLimit(Constants.k_climberCurrentLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climberSubsystem.currentDrawed() > 30);
  }
}
