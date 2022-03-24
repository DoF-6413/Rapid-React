// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnCommand extends CommandBase {
  public double toDistance;
  public double angle;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  

  public TurnCommand(DrivetrainSubsystem drivetrainSubsystem, Integer Angle) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_DrivetrainSubsystem);
    angle = Angle;
  }

  // Called when the command is nitially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.resetEncoderValue();
    toDistance = (angle > 180) ? ((angle - 360) / 180 * Math.PI)
        : (angle / 180) * Math.PI;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (angle <= 180) {
    //   if (m_DrivetrainSubsystem.LeftEncoderDistance() < toDistance)
    //     m_DrivetrainSubsystem.setRaw(0.00, 0.5);
    //   else{
    //     m_DrivetrainSubsystem.setRaw(0.00, 0.00);
    //   }
    // } else if (angle > 180) {
    //   if (m_DrivetrainSubsystem.LeftEncoderDistance() > toDistance)
    //     m_DrivetrainSubsystem.setRaw(0.00, -0.5);
    // } else {
    //   m_DrivetrainSubsystem.setRaw(0.00, 0.00);
    // }
   if  (toDistance < 0){  
     m_DrivetrainSubsystem.setRaw(0.00, -0.5);
  }else{ 
     m_DrivetrainSubsystem.setRaw(0.00, 0.5);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Distance", toDistance);
    return toDistance > 0 ? (m_DrivetrainSubsystem.LeftEncoderDistance() >= toDistance)
        : (m_DrivetrainSubsystem.LeftEncoderDistance() <= toDistance) ;

  }

  public void goClockwise(int distance) {
    System.out.println("Right" + distance);
  }

  public void goCounterClockwise(int distance) {
    System.out.println("Left" + distance);
  }
}

// 27 wheel to wheel
// 27/12/2*2*pi
// 7.07 circumference
// To turn 360 degrees go 7.07 in specific direction
// 180 = 3.53
//