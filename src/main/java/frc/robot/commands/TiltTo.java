// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TiltTo extends CommandBase {
  /** Creates a new Tilt. */
  public double Angle;
  public double StartingPosition;
  public TiltTo(double angleEndpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    Angle = angleEndpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
StartingPosition = RobotContainer.m_gyroSubsystem.getRoll();
RobotContainer.m_intakeSubsystem.stopActuators();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if(RobotContainer.m_gyroSubsystem.getRoll() > Angle){
    RobotContainer.m_intakeSubsystem.setActuatorDown(0.2);
} else if (RobotContainer.m_gyroSubsystem.getRoll() < Angle){
    RobotContainer.m_intakeSubsystem.setActuatorUp(0.2);
} else {
  RobotContainer.m_intakeSubsystem.stopActuators();
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.stopActuators();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return StartingPosition > Angle ?
    RobotContainer.m_intakeSubsystem.currentActuatorPosition() <= Angle :
    RobotContainer.m_intakeSubsystem.currentActuatorPosition() >= Angle ;
  }
}
