// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightPrep extends SequentialCommandGroup {
  /** Creates a new LimelightPrep. */
  public LimelightPrep(IndexerSubsystem index, ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> index.spinBack()),
      new WaitCommand(Constants.indexerWaitTime),
      new InstantCommand(()-> index.stopMotor()),
      new InstantCommand( () -> shoot.setSetpoint(2800)),
      new InstantCommand(() -> shoot.enable())
    );
  }
}
