// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.RunCommand;

//negative is down, positive is up
public class ClimberReset extends CommandBase {
  /** Creates a new ClimberReset. */
  public ClimberReset() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_climberSubsystem.setCurrentLimit(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   RobotContainer.m_climberSubsystem.goDownManual(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.m_climberSubsystem.setPosition();
  RobotContainer.m_climberSubsystem.stop();
   RobotContainer.m_climberSubsystem.setCurrentLimit(45);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.m_climberSubsystem.currentDrawed() > 30);
  }
}


//When the climber stops going down
//Get encoder value
//if encoder value stops changing
//then end