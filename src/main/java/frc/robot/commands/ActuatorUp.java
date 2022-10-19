// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;



public class ActuatorUp extends CommandBase {
  /** Creates a new ActuatorDown. */
  boolean up;

  private final IntakeSubsystem m_intakeSubsystem;

  // DigitalInput bottomlimitSwitch = new DigitalInput(1);

  public ActuatorUp(IntakeSubsystem intake) {
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
    m_intakeSubsystem.setAllActuatorsUp(-0.4);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopActuators();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isUp() == true;
  }
}
