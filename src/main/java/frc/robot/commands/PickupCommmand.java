// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupCommmand extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  private IntakeSubsystem m_intakeSubsystem;
  public PickupCommmand(IntakeSubsystem Intake) {
    m_intakeSubsystem = Intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new InstantCommand( () -> Intake.spinMotor()),
      new WaitCommand(Constants.oneSecond),
      new InstantCommand( () -> Intake.stopMotor())
      ); 
  }
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopMotor();
  }
}
