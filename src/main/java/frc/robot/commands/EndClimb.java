// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;


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
        // Tips Robot Back to Attach to High Bar
        // new TiltTo(-40, m_intakeSubsystem, m_gyroSubsystem),
        // new ClimberGoTo(49, m_climberSubsystem),
        // new TiltTo(-30, m_intakeSubsystem, m_gyroSubsystem),
        // new ClimberGoTo(40, m_climberSubsystem),
        // // NOW CONNECTED AT TWO POINTS
        // // BOTTOM COMMAND RELEASES FROM MID BAR
        // new ClimberGoTo(30, m_climberSubsystem),
        // parallel(new ClimberGoTo(18, m_climberSubsystem), new IntakeGoTo(-15, m_intakeSubsystem)),
        // new IntakeGoTo(-4, m_intakeSubsystem),
        // // Now attach intake to high bar
        // new ClimberGoTo(0, m_climberSubsystem),
        // new IntakeGoTo(-2, m_intakeSubsystem),
        // new WaitCommand(0.5),
        // new ClimberGoTo(7, m_climberSubsystem)

    );
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stop();
  }
}