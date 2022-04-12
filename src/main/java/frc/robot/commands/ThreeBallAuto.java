// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ThreeBallAuto extends SequentialCommandGroup {
  /** Add your docs here. */
  public ThreeBallAuto() {
    addCommands(
      new ActuatorDown(RobotContainer.m_intakeSubsystem),
      new ShootTeleopHigh( RobotContainer.m_shooterSubsystem, RobotContainer.m_indexerSubsystem, 2500),
      new MoveCommand(RobotContainer.m_drivetrainSubsystem, -3, false),
      new PickupCommmand(RobotContainer.m_intakeSubsystem), 
      new TurnAuto(RobotContainer.m_drivetrainSubsystem, RobotContainer.m_gyroSubsystem, 122),
      new MoveCommand(RobotContainer.m_drivetrainSubsystem, -10, false),
      new PickupCommmand(RobotContainer.m_intakeSubsystem),
      new TurnAuto(RobotContainer.m_drivetrainSubsystem, RobotContainer.m_gyroSubsystem, -122),
      new ShootTeleopHigh( RobotContainer.m_shooterSubsystem, RobotContainer.m_indexerSubsystem, 2800)
    );
  }
}
