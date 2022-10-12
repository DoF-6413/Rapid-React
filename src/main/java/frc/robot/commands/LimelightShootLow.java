// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** This command doesn't work, needs to be refined */
public class LimelightShootLow extends SequentialCommandGroup {
  /** Creates a new LimelightShootLow. */
  public LimelightShootLow(LimelightSubsystem light, IndexerSubsystem index, ShooterSubsystem shoot, DrivetrainSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( 
      new LimelightAim(drive, light),
      new ShootTeleopLow(shoot, index)
    );
  }
}
