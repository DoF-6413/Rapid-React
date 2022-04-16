// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LimelightFindTarget extends CommandBase {
  /** Creates a new LimelightFindTarget. */
  public LimelightFindTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_drivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(!RobotContainer.m_LimelightSubsystem.hasTarget()) {
        if(RobotContainer.m_LimelightSubsystem.getTy() > Constants.limelightMax){
            RobotContainer.m_drivetrainSubsystem.setRaw(0.5, 0.0);
        } else if(RobotContainer.m_LimelightSubsystem.getTy() < Constants.limelightMin){
            RobotContainer.m_drivetrainSubsystem.setRaw(-0.5, 0.0);
        } else { 
            RobotContainer.m_drivetrainSubsystem.setRaw(0.0, 0.0);
        }
        } 
      }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(
  ) {
    
    return (RobotContainer.m_LimelightSubsystem.getTy() > Constants.limelightMax ) ? RobotContainer.m_LimelightSubsystem.getTy() <= Constants.limelightMin : RobotContainer.m_LimelightSubsystem.getTy() >= Constants.limelightMin;
  }
}
