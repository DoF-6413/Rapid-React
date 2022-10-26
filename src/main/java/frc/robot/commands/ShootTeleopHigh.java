// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Shoots High Manually */
public class ShootTeleopHigh extends SequentialCommandGroup {
  /** Creates a new ShootTeleopHigh. */
  public ShootTeleopHigh(ShooterSubsystem shoot, IndexerSubsystem index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new InstantCommand(() -> index.stopMotor(), index), 
      parallel(new IndexerTeleop(index),new ShootHigh(shoot))
    );
  }
}
