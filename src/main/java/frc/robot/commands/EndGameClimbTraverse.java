// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndGameClimbTraverse extends SequentialCommandGroup {
  /** Creates a new EndGameClimbTraverse. */
  public EndGameClimbTraverse() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TiltTo(-40),
      new ClimberGoTo(48), 
      new TiltTo(-30),
      new ClimberGoTo(40),
      new ClimberGoTo(25),
      new intakeGoTo(-4)
    );
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.stopActuators();
    RobotContainer.m_climberSubsystem.stop();
  }
}
