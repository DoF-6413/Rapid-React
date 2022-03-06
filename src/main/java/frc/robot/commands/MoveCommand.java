// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

public class MoveCommand extends CommandBase {
  /** Creates a new MoveCommand. */

  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  public MoveCommand(DrivetrainSubsystem drivetrainSubsystem) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_DrivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.resetEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_DrivetrainSubsystem.getAvgEncocderDistance() < Constants.moveDistanceFromTarmac)
      m_DrivetrainSubsystem.setRaw(0.5, 0);
    else  m_DrivetrainSubsystem.setRaw(0.00, 0.00);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_DrivetrainSubsystem.getAvgEncocderDistance() >= Constants.moveDistanceFromTarmac);
  }
}