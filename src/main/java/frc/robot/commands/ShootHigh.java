// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
/** Runs Shooter for 4 Seconds at Upper Hub Speed
 * Note: Assumed to run parralel with IndexerTeleop Command
 */
public class ShootHigh extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  private ShooterSubsystem m_Shooter;
  public ShootHigh(ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
   m_Shooter = shoot;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new InstantCommand( () -> shoot.setSetpoint(Constants.upperHubSpeed)),
      //change to 2300 for low goal
      //change to 4250 for high goal
      new InstantCommand(() -> shoot.enable()),
      new WaitCommand(Constants.fourSeconds),
      new InstantCommand(() -> shoot.disable())
      ); 
  }

  @Override
  public void end(boolean interrupted) {
    m_Shooter.disable();
  }
}
