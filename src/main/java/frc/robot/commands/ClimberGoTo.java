// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/** This Command sets the Climber to a specific encoder value at the given speed
 * Note: All Speed Values should be Positive, Enpoint Values not Negative
 */
public class ClimberGoTo extends CommandBase {
  /** Creates a new ClimberFullRetract. */
  private double m_endpoint;
  private double startpoint;
  private double m_speed;
  private ClimberSubsystem m_climberSubsystem;

  public ClimberGoTo(double endpoint, double speed, ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_endpoint = endpoint;
    m_speed = speed;
    m_climberSubsystem = climber;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startpoint = m_climberSubsystem.getCurrentPosition();
    m_climberSubsystem.stop();
    SmartDashboard.putNumber("climber Endpoint", m_endpoint);
    SmartDashboard.putNumber("climber speed", m_speed);
    System.out.println("running initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getCurrentPosition() > m_endpoint) {
      m_climberSubsystem.goDownManual(-m_speed);
    } else if (m_climberSubsystem.getCurrentPosition() < m_endpoint) {
      m_climberSubsystem.goUpManual(m_speed);
    } else {
      m_climberSubsystem.stop();
    }

    System.out.println("running execute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
    System.out.println("ENDDDDDDDDD");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("cmdendpoint", m_endpoint);
    SmartDashboard.putNumber("cmdStartpoint", startpoint);

    return (startpoint > m_endpoint) ? m_climberSubsystem.getCurrentPosition() <= m_endpoint
        : m_climberSubsystem.getCurrentPosition() >= m_endpoint;
  }
}
