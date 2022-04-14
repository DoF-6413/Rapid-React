// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndGameClimbMid extends SequentialCommandGroup {
  /** Creates a new EndGameClimbMid. */
  public EndGameClimbMid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new IntakeGoTo(0),
      new ClimberGoTo(-1),
      new IntakeGoTo(-3.5),
      new WaitCommand(0.5),
      new ClimberGoTo(7),
      parallel (new ClimberGoTo(45), new TiltTo(-45)),
      new WaitCommand(7.5),
      new ClimberGoTo(40),
      parallel(new ClimberGoTo(20), new IntakeGoTo(-15)),
      new IntakeGoTo(-4),
      new ClimberGoTo(-1),
      new IntakeGoTo(-3.5)
      //WE ARE AT HIGH BAR
      // new WaitCommand(0.5),
      // new ClimberGoTo()
    );
  }
}
