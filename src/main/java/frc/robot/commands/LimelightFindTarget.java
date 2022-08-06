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
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightFindTarget extends CommandBase {
  /** Creates a new LimelightFindTarget. */
  public double beginningYPosition;
  public final DrivetrainSubsystem m_DrivetrainSubsystem;

  public LimelightFindTarget(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_DrivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
    beginningYPosition = RobotContainer.m_LimelightSubsystem.getTy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(RobotContainer.m_LimelightSubsystem.hasTarget()) {
        if(RobotContainer.m_LimelightSubsystem.getTy() > Constants.limelightMax){
            m_DrivetrainSubsystem.setRaw(0.5, 0.0);
        } else if(RobotContainer.m_LimelightSubsystem.getTy() < Constants.limelightMin){
            m_DrivetrainSubsystem.setRaw(-0.5, 0.0);
        } else { 
            m_DrivetrainSubsystem.setRaw(0.0, 0.0);
        }
        } 
      }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(
  ) {
    
    return (beginningYPosition > Constants.limelightMax ) ? RobotContainer.m_LimelightSubsystem.getTy() <= Constants.limelightMax : RobotContainer.m_LimelightSubsystem.getTy() >= Constants.limelightMin;
  }
}
