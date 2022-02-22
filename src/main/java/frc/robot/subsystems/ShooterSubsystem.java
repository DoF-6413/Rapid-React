// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Talon FX Imports
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;

import java.security.KeyStore.Entry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.*;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.wpilibj.Encoder;

public class ShooterSubsystem extends PIDSubsystem {
  /** Creates a new ExampleSubsystem. */
  // Example usage of a TalonFX motor controller
  TalonFX leftShooterMotor = new TalonFX(Constants.shooterID[0]); // creates a new TalonFX with ID 0
  TalonFX rightShooterMotor = new TalonFX(Constants.shooterID[1]);
  // private final Encoder m_shooteEncoder = new Encoder (
  // Constants.shooterID[0]
  // )

  double s = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable datatable = inst.getTable("SmartDashboard");
  NetworkTableEntry sEntry = datatable.getEntry("setpoint");

  public ShooterSubsystem() {
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));
    // getController().setTolerance(Constants.kShooterToleranceRPS);
    // leftShooterMotor.setVelocityPerPulse(Constants.kEncoderDistancePerPulse);

    SmartDashboard.putNumber("setpoint", 0);
    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    // NetworkTableEntry setpoint = datatable.getEntry("setpoint");
    inst.startClientTeam(6413);

    rightShooterMotor.setInverted(true);
    // Set right follow motors
    rightShooterMotor.follow(leftShooterMotor);
    setSetpoint((double) 4000);

  }

  @Override
  public void useOutput(double output, double setpoint) {

    SmartDashboard.putNumber("USE OUTPUT!: ", setpoint);
    leftShooterMotor.set(TalonFXControlMode.Velocity, (setpoint - output));
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("GET MEASUREMENT: ", leftShooterMotor.getSelectedSensorPosition());
    return leftShooterMotor.getSelectedSensorPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void periodic() {;
    SmartDashboard.putNumber("Right RPM= ", rightShooterMotor.getSelectedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Left RPM= ", leftShooterMotor.getSelectedSensorVelocity() * 600 / 2048);
    // This method will be called once per scheduler run

    sEntry.setDouble(s);
    s += 1.0;
  }

  // rpm = velocity*600/2048

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void spinMotor() {
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, Constants.shooterSpeed); // runs the motor at 25% power
  }

  public void stopMotor() {
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, 0); // runs the motor at 0% power

  }

  // public void setFlywheelActiveSpeed(){
  // leftShooterMotor.set(pid.calculate(encoder.getDistance(), setpoint));

  // }

}
