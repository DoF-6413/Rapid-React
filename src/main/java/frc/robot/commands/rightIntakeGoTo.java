// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class rightIntakeGoTo extends CommandBase {
  /** Creates a new IntakeGoTo. */
  public double endpoint;
 //tracking initial start point so we can compare during climbing to see when to finish
  public double leftStartpoint;
  public double rightStartpoint;
  public rightIntakeGoTo(double Endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    endpoint = -Endpoint;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_intakeSubsystem.stopActuators();
    rightStartpoint = RobotContainer.m_intakeSubsystem.currentRightActuatorPosition();
    RobotContainer.m_intakeSubsystem.climbingCurrentLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

      if(RobotContainer.m_intakeSubsystem.currentRightActuatorPosition() < endpoint){
          RobotContainer.m_intakeSubsystem.setRightActuatorDown(Constants.actuatorsSpeed);
          }
      else if (RobotContainer.m_intakeSubsystem.currentRightActuatorPosition() > endpoint){
          RobotContainer.m_intakeSubsystem.setRightActuatorUp(Constants.actuatorsSpeed);
        } 
      else {
          RobotContainer.m_intakeSubsystem.stopRightActuator();
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
    return (rightStartpoint < endpoint) ?
    (RobotContainer.m_intakeSubsystem.currentRightActuatorPosition() >= endpoint) :
    (RobotContainer.m_intakeSubsystem.currentRightActuatorPosition() <= endpoint);
  }
}
