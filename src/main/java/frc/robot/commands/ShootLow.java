// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
/** Runs Shooter for 4 Seconds at Low Hub RPM
 * Note: Assumed to run parralel with IndexerTeleop Command
 */
public class ShootLow extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  private ShooterSubsystem m_Shooter;
  public ShootLow(ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Shooter = shoot;
    addCommands( 
      new InstantCommand( () -> shoot.setSetpoint(Constants.lowerHubSpeed)),
      new InstantCommand(() -> shoot.enable()),
      new WaitCommand(Constants.twoAndHalfSeconds),
      new InstantCommand(() -> shoot.disable())
      ); 
  }
  @Override
  public void end(boolean interrupted) {
    m_Shooter.disable();
  }
}
