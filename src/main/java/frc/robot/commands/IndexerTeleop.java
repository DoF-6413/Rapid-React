// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

/**
 * Runs indexer back to give the shooter time to ramp up, then runs indexer
 * forwards to shoot
 * Note: Assumed to run parralel with ShootHigh Command
 */
public class IndexerTeleop extends SequentialCommandGroup {
  /** Creates a new IndexerTeleop. */
  public IndexerTeleop(IndexerSubsystem index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> index.spinBack()),
        new WaitCommand(Constants.indexerWaitTime),
        new InstantCommand(() -> index.spinMotor()),
        new WaitCommand(Constants.oneSecond),
        new InstantCommand(() -> index.stopMotor())
    );
  }
}
