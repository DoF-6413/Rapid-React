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
      new ClimberGoTo(48), 
      new TiltTo(-30),
      new ClimberGoTo(40),
      //NOW CONNECTED AT TWO POINTS
      //BOTTOM COMMAND RELEASES FROM MID BAR
      new ClimberGoTo(25),
      parallel (new ClimberGoTo(20), new IntakeDown()),
      new IntakeGoTo(-15),
      new IntakeGoTo(-4),
      new ClimberGoTo(0),
      new IntakeGoTo(-2),
      new ClimberGoTo(7)
      //WE ARE AT HIGH BAR
      // new WaitCommand(0.5),
      // new ClimberGoTo(47), // full exstention
      // new WaitCommand(2.5),
      // new TiltTo(-25),
      // new WaitCommand(2.5),
      // new TiltTo(-30),
      // new WaitCommand(1),
      // new ClimberGoTo(25),
      
      //new IntakeGoTo(-2)
    );
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.stopActuators();
    RobotContainer.m_climberSubsystem.stop();
  }
}
