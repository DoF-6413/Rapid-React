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
public class EndGameClimbHigh extends SequentialCommandGroup {
  /** Creates a new EndGameClimbHigh. */
  public EndGameClimbHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TiltTo(-40),
      new ClimberGoTo(49), 
      new TiltTo(-30),
      new ClimberGoTo(40),
      //NOW CONNECTED AT TWO POINTS
      //BOTTOM COMMAND RELEASES FROM MID BAR
      new ClimberGoTo(30),
      parallel( new ClimberGoTo(18), new intakeGoTo(-15)),
      new intakeGoTo(-4),
      new ClimberGoTo(0),
      new intakeGoTo(-2),
      new WaitCommand(0.5),
      new ClimberGoTo(7)
     
    );
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.stopActuators();
    RobotContainer.m_climberSubsystem.stop();
  }
}
