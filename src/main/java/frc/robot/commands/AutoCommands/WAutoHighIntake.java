// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ActuatorDown;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.PickupCommmand;
import frc.robot.commands.ShootHigh;
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
public class WAutoHighIntake extends SequentialCommandGroup {
  /** Creates a new OttoCommand. */
  public WAutoHighIntake(DrivetrainSubsystem drive, ShooterSubsystem shoot, IndexerSubsystem Index, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()); 
    addCommands(
      new ActuatorDown(intake),
      new MoveCommand(drive, -3, false),
      new InstantCommand(()-> System.out.println("YOU ARE RUNNING IN AUTO LOL")),
      //new MoveCommand(drive, 4, true),
      parallel(new PickupCommmand(intake)), 
      new MoveCommand(drive, 1, true),
      parallel( new ShootHigh(shoot), new IndexerCommand(Index))
    );
  }
}

