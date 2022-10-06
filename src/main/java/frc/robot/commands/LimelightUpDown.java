// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This command moves the robot forward and backward utill in ideal distance for high goal shot
 */

public class LimelightUpDown extends CommandBase {
  /** Creates a new LimelightFindTarget. */
  private double beginningYPosition;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;

  public LimelightUpDown(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_LimelightSubsystem =limelightSubsystem;
    addRequirements(m_DrivetrainSubsystem);
    addRequirements(m_LimelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
    beginningYPosition = m_LimelightSubsystem.getTy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_LimelightSubsystem.hasTarget()) {
        if(m_LimelightSubsystem.getTy() > Constants.limelightYMax){
            m_DrivetrainSubsystem.setRaw(0.5, 0.0);
        } else if(m_LimelightSubsystem.getTy() < Constants.limelightYMin){
            m_DrivetrainSubsystem.setRaw(-0.5, 0.0);
        } else { 
            m_DrivetrainSubsystem.setRaw(0.0, 0.0);
        }
        } else { 
          m_DrivetrainSubsystem.setRaw(0.0, 0.0);
      }
      }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (beginningYPosition > Constants.limelightYMax ) ? 
        m_LimelightSubsystem.getTy() <= Constants.limelightYMax :
        m_LimelightSubsystem.getTy() >= Constants.limelightYMin;
  }
}
