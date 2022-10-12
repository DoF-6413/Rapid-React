// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/**
 * Moves the intake to a given angle
 * Note: Assumes we are on one of the climber bars
 */
public class TiltTo extends CommandBase {
  /** Creates a new Tilt. */
  private double Angle;
  private double StartingPosition;
  private IntakeSubsystem m_intakeSubsystem;
  private GyroSubsystem m_gyroSubsystem;

  public TiltTo(double angleEndpoint, IntakeSubsystem intake, GyroSubsystem gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    Angle = angleEndpoint;
    m_intakeSubsystem = intake;
    m_gyroSubsystem = gyro;
    addRequirements(m_intakeSubsystem, m_gyroSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StartingPosition = m_gyroSubsystem.getRoll();
    m_intakeSubsystem.stopActuators();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_gyroSubsystem.getRoll() > Angle) {
      m_intakeSubsystem.setRightActuatorDown(0.2);
      m_intakeSubsystem.setLeftActuatorDown(0.2);
    } else if (m_gyroSubsystem.getRoll() < Angle) {
      m_intakeSubsystem.setRightActuatorUp(0.2);
      m_intakeSubsystem.setLeftActuatorUp(0.2);
    } else {
      m_intakeSubsystem.stopActuators();
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
    return StartingPosition > Angle ? m_gyroSubsystem.getRoll() <= Angle : m_gyroSubsystem.getRoll() >= Angle;
  }
}
