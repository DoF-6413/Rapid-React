// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Run Climber to Put the Robot on Traversal Bar
 * Note: This assumes the robot has already ran BeginClimb and is in proper position
 */
public class EndClimb extends SequentialCommandGroup {
  /** Creates a new EndGameClimbHigh. */
  private ClimberSubsystem m_climberSubsystem;

  public EndClimb(ClimberSubsystem climber) {
    m_climberSubsystem = climber;
    addRequirements(m_climberSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ClimberGoTo(Constants.k_climberBottom, Constants.k_climberMaxUp ,  m_climberSubsystem),
      new InstantCommand(() -> m_climberSubsystem.runStingerMotor()),
       new WaitCommand(Constants.oneSecond),
       new InstantCommand(() -> m_climberSubsystem.stopStingerMotor())
      //  new InstantCommand(() -> m_climberSubsystem.goUpManual(Constants.k_climberMaxUp)),
      //  new WaitCommand(Constants.oneSecond), //TODO: Change time
      //  new InstantCommand(() -> m_climberSubsystem.stop())

    );
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
  }
}
