// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance; 
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoShootToCorner extends SequentialCommandGroup {
  /** Creates a new AutoShootToCorner. 
  *This auto drives back, gathers another one of our cargo, and shoots the pre-loaded cargo and the collected cargo
  *Then the auto turns and drives back to gather the opposite alliance cargo 1
  *Then the auto turns and drives back to gather opposite alliance cargo 2
  *Finally it turns and drives back to the corner of our alliance hangar and low shoots the balls into the corner
  */

  public AutoShootToCorner(DrivetrainSubsystem drive, ShooterSubsystem shoot, IndexerSubsystem Index, IntakeSubsystem intake, GyroSubsystem gyro) {
    addCommands(
      new ActuatorDown(intake),
      new MoveCommand(drive, -3, false),
      new PickupCommmand(intake),
      new MoveCommand(drive, 3, true),
      parallel( new ShootHigh(shoot), new IndexerCommand(Index)),
      new TurnAuto(new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D), drive, gyro, 34),
      new MoveCommand(drive, -4, false),
      new PickupCommmand(intake),
      new TurnAuto(new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D), drive, gyro, -118),
      new MoveCommand(drive, -8, false),
      new PickupCommmand(intake),
      new MoveCommand(drive, -2, false),
      new TurnAuto(new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D), drive, gyro, 23),
      new MoveCommand(drive, 10, true),
      parallel( new ShootLow(shoot), new IndexerCommand(Index))
    );
  }
}
