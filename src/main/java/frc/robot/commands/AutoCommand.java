// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance; 
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {
  /** Creates a new OttoCommand. */
  public AutoCommand(DrivetrainSubsystem drive, ShooterSubsystem shoot, IndexerSubsystem Index, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()); 
    addCommands(
      new InstantCommand(()-> System.out.println("YOU ARE RUNNING IN AUTO LOL")),
      //comment out above for high goal
      new MoveCommand(drive, 3, true),
      parallel( new Shoot(shoot), new IndexerCommand(Index)),
      new MoveCommand(drive, -8, false)
      // change above to -8 for low goal
      // change above to -4 for high hoal
      //new PickupCommmand(intake)
    );
  }
}

