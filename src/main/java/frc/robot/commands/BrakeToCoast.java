// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BrakeToCoast extends CommandBase {
  /** Creates a new brakeToCoast. */
  public BrakeToCoast() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intakeSubsystem.intakeLeftActuator.setIdleMode(CANSparkMax.IdleMode.kCoast);
    RobotContainer.m_intakeSubsystem.intakeRightActuator.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.m_intakeSubsystem.intakeLeftActuator.getIdleMode() == CANSparkMax.IdleMode.kCoast) &&
    (RobotContainer.m_intakeSubsystem.intakeRightActuator.getIdleMode() == CANSparkMax.IdleMode.kCoast);

  }
}
