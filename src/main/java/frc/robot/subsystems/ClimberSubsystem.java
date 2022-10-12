// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

/**
 * This is the climber subsystem
 */
public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX climberMotor = new TalonFX(Constants.ClimberID);

  public ClimberSubsystem() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit.enable = true;
    config.statorCurrLimit.currentLimit = 50;
    climberMotor.configAllSettings(config);

  }

  public void goDownManual(double speed) {
    climberMotor.set(TalonFXControlMode.PercentOutput, speed);
    climberPosition();
  }

  public void goUpManual(double speed) {
    climberMotor.set(TalonFXControlMode.PercentOutput, speed);
    climberPosition();
  }

  public void stop() {
    climberMotor.set(TalonFXControlMode.PercentOutput, 0); // runs the motor at 0% power
    climberPosition();
  }

  public void climberPosition() {
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getSelectedSensorPosition() / 6380);
    SmartDashboard.putNumber("Climber Current", climberMotor.getStatorCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getCurrentPosition() {
    return climberMotor.getSelectedSensorPosition() / 6380;
  }

  public void setCurrentLimit(double Current) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit.currentLimit = Current;
    climberMotor.configAllSettings(config);
  }

  public void setPosition() {
    climberMotor.setSelectedSensorPosition(0);
  }

  public double currentDrawed() {
    return climberMotor.getStatorCurrent();
  }

  public static boolean getLeftTriggerActive() {
    return (RobotContainer.m_xbox.getLeftTriggerAxis() > 0);
  }

  public static boolean getRightTriggerActive() {
    return (RobotContainer.m_xbox.getRightTriggerAxis() > 0);
  }
}
