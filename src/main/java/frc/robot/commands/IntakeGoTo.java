// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class IntakeGoTo extends CommandBase {
  /** Creates a new IntakeGoTo. */
  public double endpoint;
  public double startpoint;
  public IntakeGoTo(double Endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = Endpoint;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_intakeSubsystem.stopActuators();
    startpoint = RobotContainer.m_intakeSubsystem.currentActuatorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_intakeSubsystem.currentActuatorPosition() > endpoint){
      RobotContainer.m_intakeSubsystem.setActuatorDown(Constants.actuatorsSpeed);
      } else if (RobotContainer.m_intakeSubsystem.currentActuatorPosition() < endpoint){
        RobotContainer.m_intakeSubsystem.setActuatorUp(Constants.actuatorsSpeed);
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
    return (startpoint > endpoint) ?
    RobotContainer.m_intakeSubsystem.currentActuatorPosition() <= endpoint :
    RobotContainer.m_intakeSubsystem.currentActuatorPosition() >= endpoint ;
  }
}
