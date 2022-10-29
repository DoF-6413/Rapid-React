// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Turns the robot to a specific angle */
public class TurnAuto extends PIDCommand {
  /** Creates a new TurnAuto. */
  private GyroSubsystem m_gyroSubsystem;
  private DrivetrainSubsystem m_drivetrainSubsystem;

  public TurnAuto(DrivetrainSubsystem drivetrainSubsystem, GyroSubsystem gyroSubsystem, double targetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
        new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D),
        // Close loop on heading
        gyroSubsystem::getAngle,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drivetrainSubsystem.setRaw(Constants.k_stopMotor, output / 180), // divide by 180 to maybe scale angle to be between minus
                                                               // 1 and 1
        // Require the drive
        drivetrainSubsystem);

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_gyroSubsystem = gyroSubsystem;
    addRequirements(m_drivetrainSubsystem, m_gyroSubsystem);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.K_TURN_TOLERANCE_DEG, Constants.K_TURN_RATE_TOLERANCE_DEG_PER_SEC);
    getController().setSetpoint(targetAngleDegrees);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyroSubsystem.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.setRaw(Constants.k_stopMotor, (getController().calculate(m_gyroSubsystem.getAngle()) / 100));
    SmartDashboard.putNumber("Calculate Results", getController().calculate(m_gyroSubsystem.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.setRaw(Constants.k_stopMotor, Constants.k_stopMotor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out
    //     .println("Is Finished Return " + getController().atSetpoint() + " setpoint " + getController().getSetpoint());
    // return getController().atSetpoint();

    if (
            (
              (m_gyroSubsystem.getAngle() >= getController().getSetpoint()) &
              (m_gyroSubsystem.getAngle() <= (getController().getSetpoint() + Constants.K_TURN_TOLERANCE_DEG))
            ) ||
            (
              (m_gyroSubsystem.getAngle() >= getController().getSetpoint()) &
              (m_gyroSubsystem.getAngle() <= (getController().getSetpoint() - Constants.K_TURN_TOLERANCE_DEG))
            )
      ) {
        return true;
      } else {
        return false;
      }

    
  }
}
