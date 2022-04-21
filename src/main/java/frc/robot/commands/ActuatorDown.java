// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;


public class ActuatorDown extends CommandBase {
  /** Creates a new ActuatorDown. */
  boolean down;

  private final IntakeSubsystem m_intakeSubsystem;

  // DigitalInput bottomlimitSwitch = new DigitalInput(1);

  public ActuatorDown(IntakeSubsystem intake) {
    m_intakeSubsystem = intake;
    addRequirements(m_intakeSubsystem);// Use addRequirements() here to declare subsystem dependencies.
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setAllActuatorsDown(0.4);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopActuators();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_intakeSubsystem.isDown() == true;
  }
}
