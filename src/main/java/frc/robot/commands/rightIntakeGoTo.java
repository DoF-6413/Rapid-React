// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

/** Moves the rights intake side to specific encoder value */
public class rightIntakeGoTo extends CommandBase {
  /** Creates a new IntakeGoTo. */
  private double endpoint;
  // tracking initial start point so we can compare during climbing to see when to
  // finish
  private double rightStartpoint;
  private IntakeSubsystem m_intakeSubsystem;

  public rightIntakeGoTo(double Endpoint, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = -Endpoint;
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.stopActuators();
    rightStartpoint = m_intakeSubsystem.currentRightActuatorPosition();
    m_intakeSubsystem.climbingCurrentLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.currentRightActuatorPosition() < endpoint) {
      m_intakeSubsystem.setRightActuatorDown(Constants.actuatorsSpeed);
    } else if (m_intakeSubsystem.currentRightActuatorPosition() > endpoint) {
      m_intakeSubsystem.setRightActuatorUp(Constants.actuatorsSpeed);
    } else {
      m_intakeSubsystem.stopRightActuator();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopActuators();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (rightStartpoint < endpoint) ? (m_intakeSubsystem.currentRightActuatorPosition() >= endpoint)
        : (m_intakeSubsystem.currentRightActuatorPosition() <= endpoint);
  }
}
