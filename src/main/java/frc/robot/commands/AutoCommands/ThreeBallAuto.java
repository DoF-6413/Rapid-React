// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnAuto;
import frc.robot.Constants;
import frc.robot.commands.LLAimAndShoot;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.PickupCommmand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto(DrivetrainSubsystem drive, ShooterSubsystem shoot, IndexerSubsystem Index, IntakeSubsystem intake, GyroSubsystem gyro, LimelightSubsystem light
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WAutoHighIntake(drive, shoot, Index, intake),
      new TurnAuto(drive, gyro, Constants.k_firstTurnAuto),
      parallel (new MoveCommand(drive, Constants.k_moveThreeBall, false), new PickupCommmand(intake)),
      new PickupCommmand(intake),
      parallel (new TurnAuto(drive, gyro, Constants.k_secondTurnAuto), new PickupCommmand(intake)),
      new LLAimAndShoot(light, Index, shoot, drive)
    );
  }
}
