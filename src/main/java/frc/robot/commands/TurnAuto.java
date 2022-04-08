// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnAuto extends PIDCommand {
  /** Creates a new TurnAuto. */
private PIDController m_PidController;

private DrivetrainSubsystem m_drivetrain;
  public TurnAuto(PIDController pidControl, DrivetrainSubsystem drivetrainSubsystem, GyroSubsystem gyroSubsystem, double targetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    super(
      pidControl,
      // Close loop on heading
      gyroSubsystem::getAngle,
      // Set reference to target
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> drivetrainSubsystem.autoDrive(0, output), // divide by 180 to maybe scale angle to be between minus 1 and 1
      // Require the drive
      drivetrainSubsystem);
  
    m_PidController = pidControl;

    m_drivetrain = drivetrainSubsystem;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.K_TURN_TOLERANCE_DEG, Constants.K_TURN_RATE_TOLERANCE_DEG_PER_SEC);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      RobotContainer.m_gyroSubsystem.resetYaw();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartDashboard.putNumber("TurnAngle", RobotContainer.m_gyroSubsystem.getAngle());
      SmartDashboard.putNumber("PID Calculate", m_PidController.calculate(RobotContainer.m_gyroSubsystem.getAngle()));
      m_drivetrain.autoDrive( 0, m_PidController.calculate(RobotContainer.m_gyroSubsystem.getAngle()));  
      
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setRaw(0.0, 0.0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return getController().atSetpoint();
  }
}
