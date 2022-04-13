// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * This autonomous command does the following actions:
 * 
 * Shoot the preloaded ball into the high goal
 * Drop the intake and back up to pick up the ball behind the robot
 * Turn towards a 3rd ball
 * Move to pick up the 3rd ball
 * Turn towards the tower 
 * Score both balls in the high goal
 */
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Add your docs here. */
  public ThreeBallAuto() {
    addCommands(
      new ActuatorDown(RobotContainer.m_intakeSubsystem),
      new ShootTeleopHigh( RobotContainer.m_shooterSubsystem, RobotContainer.m_indexerSubsystem, Constants.upperHubSpeedAuto1 ),
      new MoveCommand(RobotContainer.m_drivetrainSubsystem, -3, false),
      new PickupCommmand(RobotContainer.m_intakeSubsystem), 
      new TurnAuto(RobotContainer.m_drivetrainSubsystem, RobotContainer.m_gyroSubsystem, 122),
      new MoveCommand(RobotContainer.m_drivetrainSubsystem, -9, false),
      new PickupCommmand(RobotContainer.m_intakeSubsystem),
      new TurnAuto(RobotContainer.m_drivetrainSubsystem, RobotContainer.m_gyroSubsystem, -122),
      new ShootTeleopHigh( RobotContainer.m_shooterSubsystem, RobotContainer.m_indexerSubsystem, Constants.upperHubSpeedAuto2 )   // This other speed is untested!
    );
  }
}
