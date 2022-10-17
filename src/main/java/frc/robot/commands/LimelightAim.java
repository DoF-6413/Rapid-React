// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/**
 * This command moves the robot left and right utill in ideal angle for high
 * goal shot
 */

public class LimelightAim extends CommandBase {
  /** Creates a new LimelightLeftRight. */
  private double beginningXPosition;
  private double beginningYPosition;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;
  private ShooterSubsystem m_ShooterSubsystem;
  private IndexerSubsystem m_IndexerSubsystem;

  public LimelightAim(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IndexerSubsystem = indexerSubsystem;
    addRequirements(m_DrivetrainSubsystem);
    addRequirements(m_LimelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
    beginningXPosition = m_LimelightSubsystem.getTx();
    beginningYPosition = m_LimelightSubsystem.getTy();
    new LimelightPrep(m_IndexerSubsystem, m_ShooterSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightSubsystem.hasTarget()) {
      double leftStick = 0.0;
      double rightStick = 0.0;

      if (m_LimelightSubsystem.getTx() > Constants.limelightXMax) {
        rightStick = 0.5;
    
      } else if (m_LimelightSubsystem.getTx() < Constants.limelightXMin) {
        rightStick = -0.5;
      } 

      if (m_LimelightSubsystem.getTy() > Constants.limelightYMax) {
        leftStick = 0.5;
      } else if (m_LimelightSubsystem.getTy() < Constants.limelightYMin) {
        leftStick = -0.5;
      }

      m_DrivetrainSubsystem.setRaw(leftStick, rightStick);
    } // if hasTarget
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.setRaw(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
Boolean xDone =  (beginningXPosition > Constants.limelightXMax) ?
       m_LimelightSubsystem.getTx() <= Constants.limelightXMax
      : m_LimelightSubsystem.getTx() >= Constants.limelightXMin;

      Boolean yDone = (beginningYPosition > Constants.limelightYMax ) ? 
      m_LimelightSubsystem.getTy() <= Constants.limelightYMax :
      m_LimelightSubsystem.getTy() >= Constants.limelightYMin;

      return ( xDone && yDone );
  }
}
