// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class ClimberGoTo extends CommandBase {
  /** Creates a new ClimberFullRetract. */
  public double endpoint;
  public ClimberGoTo(double Endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = Endpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_climberSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
if(RobotContainer.m_climberSubsystem.getCurrentPosition() > endpoint){
RobotContainer.m_climberSubsystem.goDownManual(Constants.climberSpeedDown);
} else if (RobotContainer.m_climberSubsystem.getCurrentPosition() < endpoint){
  RobotContainer.m_climberSubsystem.goUpManual(Constants.climberSpeedUp);
} else {
  RobotContainer.m_climberSubsystem.stop();
}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.m_climberSubsystem.getCurrentPosition() > endpoint) ?
    RobotContainer.m_climberSubsystem.getCurrentPosition() <= endpoint :
    RobotContainer.m_climberSubsystem.getCurrentPosition() >= endpoint ;
  }
}
