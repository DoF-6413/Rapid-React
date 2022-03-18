// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnCommand extends CommandBase {
  public double toDistance;
  public boolean goesClockwise;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;

  public TurnCommand(DrivetrainSubsystem drivetrainSubsystem, Integer Angle, boolean Direction) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_DrivetrainSubsystem);
    toDistance = Angle * Constants.turningRadius ;
    goesClockwise = Direction;




    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DrivetrainSubsystem.resetEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (goesForward) {
    //   goForward(toDistance);
    // } else if (!goesForward) {
    //  goBackwards(toDistance);
    // } else {
    //   m_DrivetrainSubsystem.setRaw(0.00, 0.00);

    // }
    if (goesClockwise) {
      if (m_DrivetrainSubsystem.getAvgEncocderDistance() < toDistance)
        m_DrivetrainSubsystem.setRaw(0.00, 0.5);
      else{
        m_DrivetrainSubsystem.setRaw(0.00, 0.00);
      }
    } else if (!goesClockwise) {
      if (m_DrivetrainSubsystem.getAvgEncocderDistance() > toDistance)
        m_DrivetrainSubsystem.setRaw(0.00, -0.5);
    } else {
      m_DrivetrainSubsystem.setRaw(0.00, 0.00);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return goesClockwise ?  (m_DrivetrainSubsystem.LeftEncoderDistance() >= toDistance) : (m_DrivetrainSubsystem.LeftEncoderDistance() <= toDistance);
  }


  public void goClockwise(int distance){
System.out.println("Right" + distance);
  }

  public void goCounterClockwise(int distance){
System.out.println("Left" + distance);
  }
}

//27 wheel to wheel
//27/12/2*2*pi
//7.07 circumference
//To turn 360 degrees go 7.07 in specific direction
//180 = 3.53
//