// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants;

/** This Command sets the Climber to a specific encoder value (0-49, Bottom to Top) */
public class ClimberGoTo extends CommandBase {
  /** Creates a new ClimberFullRetract. */
  private double endpoint;
  private double startpoint;
  private ClimberSubsystem m_climberSubsystem;

  public ClimberGoTo(double Endpoint, ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = Endpoint;
    m_climberSubsystem = climber;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startpoint = m_climberSubsystem.getCurrentPosition();
    m_climberSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getCurrentPosition() > endpoint) {
      m_climberSubsystem.goDownManual(Constants.climberSpeedDown);
    } else if (m_climberSubsystem.getCurrentPosition() < endpoint) {
      m_climberSubsystem.goUpManual(Constants.climberSpeedUp);
    } else {
      m_climberSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("cmdEndpoint", endpoint);
    SmartDashboard.putNumber("cmdStartpoint", startpoint);

    return (startpoint > endpoint) ? m_climberSubsystem.getCurrentPosition() <= endpoint
        : m_climberSubsystem.getCurrentPosition() >= endpoint;
  }
}
