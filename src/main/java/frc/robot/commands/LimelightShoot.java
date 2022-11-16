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
/**Shoots Cargo after Aligning in Proper Position 
 * Note: This assumes the setpoint is already set and the shooter is enabled
*/
public class LimelightShoot extends SequentialCommandGroup {
  /** Creates a new LimelightPost. */
  public LimelightShoot(IndexerSubsystem index, ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> index.spinMotor()),
      new WaitCommand(Constants.oneSecond),
      new InstantCommand(()-> index.stopMotor()),
      new WaitCommand(Constants.fourSeconds),
      new InstantCommand(() -> shoot.disable())
    );
  }
}
