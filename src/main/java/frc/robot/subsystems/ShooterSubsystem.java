// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Talon FX Imports
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ShooterSubsystem extends PIDSubsystem {
  private TalonFX leftShooterMotor = new TalonFX(Constants.shooterID[1]); // creates a new TalonFX with ID 0
  private double m_setpoint = 2000;
  private TalonFX rightShooterMotor = new TalonFX(Constants.shooterID[0]);

  public ShooterSubsystem() {
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));
    getController().setTolerance(Constants.k_shooterTolerance);

    // SmartDashboard.putNumber("setpoint", m_setpoint);
    rightShooterMotor.setInverted(true);
    // Set right follow motors
    rightShooterMotor.follow(leftShooterMotor);
    setSetpoint(m_setpoint);
  }
  
  @Override
  public void useOutput(double output, double setpoint) {
    m_setpoint = setpoint;
    SmartDashboard.putNumber("setpoints", setpoint);
    SmartDashboard.putNumber("output", leftShooterMotor.getSelectedSensorVelocity() * 600 / 2048);
    System.out.println("setpoint" + setpoint);
    System.out.println("output" + leftShooterMotor.getSelectedSensorVelocity() * 600 / 2048);

    leftShooterMotor.set(TalonFXControlMode.PercentOutput, setpoint / 6380);

  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("GET MEASUREMENT: ", leftShooterMotor.getSensorCollection().getIntegratedSensorVelocity());
    return leftShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void LowerHub() {
    setSetpoint(Constants.lowerHubSpeed); // runs the motor at deez nuts power
  }

  public void UpperHub() {
    setSetpoint(Constants.upperHubSpeed); // runs the motor at your mom power
  }

  public void stopMotor() {
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, 0); // runs the motor at 0% power

  }

}
