// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

/** Moves the left intake side to specific encoder value */
public class leftIntakeGoTo extends CommandBase {
  /** Creates a new IntakeGoTo. */
  private double endpoint;
  // tracking initial start point so we can compare during climbing to see when to
  // finish
  private double leftStartpoint;
  private IntakeSubsystem m_intakeSubsystem;

  public leftIntakeGoTo(double Endpoint, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = Endpoint;
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.stopActuators();
    leftStartpoint = m_intakeSubsystem.currentLeftActuatorPosition();
    m_intakeSubsystem.climbingCurrentLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakeSubsystem.currentLeftActuatorPosition() > endpoint) {
      m_intakeSubsystem.setLeftActuatorDown(Constants.actuatorsSpeed);
    } else if (m_intakeSubsystem.currentLeftActuatorPosition() < endpoint) {
      m_intakeSubsystem.setLeftActuatorUp(Constants.actuatorsSpeed);
    } else {
      m_intakeSubsystem.stopLeftActuator();
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
    return (leftStartpoint > endpoint) ? 
      (m_intakeSubsystem.currentLeftActuatorPosition() <= endpoint):
      (m_intakeSubsystem.currentLeftActuatorPosition() >= endpoint);
  }
}
