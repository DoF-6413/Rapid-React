// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimbActuators extends PIDCommand {
  /** Creates a new TurnAuto. */
  public ClimbActuators(IntakeSubsystem intakeSubsystem, GyroSubsystem gyroSubsystem, double targetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
  
    super(
      new PIDController(Constants.K_CHASSIS_TURN_P, Constants.K_CHASSIS_TURN_I, Constants.K_CHASSIS_TURN_D), 
      // Close loop on heading
      gyroSubsystem::getRoll,
      // Set reference to target
      targetAngleDegrees,
      // Pipe output to turn robot
      output -> intakeSubsystem.setActuatorDown(2), // divide by 180 to maybe scale angle to be between minus 1 and 1
      // Require the drive
      intakeSubsystem);
      
      
      
      // Set the controller to be continuous (because it is an angle controller)
      getController().enableContinuousInput(-180, 180);
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      getController()
      .setTolerance(Constants.K_TURN_TOLERANCE_DEG, Constants.K_TURN_RATE_TOLERANCE_DEG_PER_SEC);
      getController().setSetpoint(targetAngleDegrees);
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
      System.out.println("Execute Works");
      RobotContainer.m_drivetrainSubsystem.current();
      //SmartDashboard.putNumber("PID Calculate", m_PidController.calculate(RobotContainer.m_gyroSubsystem.getAngle()));
      RobotContainer.m_drivetrainSubsystem.setRaw( 0, (getController().calculate(RobotContainer.m_gyroSubsystem.getAngle())/100));  
      SmartDashboard.putNumber("Calculate Results", getController().calculate(RobotContainer.m_gyroSubsystem.getAngle()));
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
     RobotContainer.m_drivetrainSubsystem.setRaw(0, 0);
     //RobotContainer.m_gyroSubsystem.resetYaw();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      System.out.println("Is Finished Return " + getController().atSetpoint() + " setpoint " + getController().getSetpoint());
      return getController().atSetpoint();
  }
}
