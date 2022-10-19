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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

/**
 * This is the climber subsystem
 */
public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  TalonFX topLiftMotor;
  TalonFX bottomLiftMotor;
  CANSparkMax stingerMotor;

  public ClimberSubsystem() {
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit.enable = true;
    config.statorCurrLimit.currentLimit = Constants.k_climberCurrentLimit;
    
    topLiftMotor = new TalonFX(Constants.ClimberID[Constants.k_topLiftMotor]);
    bottomLiftMotor = new TalonFX(Constants.ClimberID[Constants.k_bottomLiftMotor]);
    topLiftMotor.configAllSettings(config);
    bottomLiftMotor.configAllSettings(config);
    bottomLiftMotor.follow(topLiftMotor);

    stingerMotor =  new CANSparkMax(Constants.ClimberID[Constants.k_stingerMotor], MotorType.kBrushless);
  }

  public void goDownManual(double speed) {
    topLiftMotor.set(TalonFXControlMode.PercentOutput, speed);
    
  }

  public void goUpManual(double speed) {
    topLiftMotor.set(TalonFXControlMode.PercentOutput, speed);
    
  }

  public void stop() {
    topLiftMotor.set(TalonFXControlMode.PercentOutput, 0); // runs the motor at 0% power
    
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Climber Encoder", topLiftMotor.getSelectedSensorPosition() / 6380);
    SmartDashboard.putNumber("Climber Current", topLiftMotor.getStatorCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDashboard();
  }

  public double getCurrentPosition() {
    return topLiftMotor.getSelectedSensorPosition() / 6380;
  }

  public void setCurrentLimit(double Current) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit.currentLimit = Current;
    topLiftMotor.configAllSettings(config);
  }

  public void setPosition() {
    topLiftMotor.setSelectedSensorPosition(0);
  }

  public double currentDrawed() {
    return topLiftMotor.getStatorCurrent();
  }

  public static boolean getLeftTriggerActive() {
    return (RobotContainer.m_xbox.getLeftTriggerAxis() > 0);
  }

  public static boolean getRightTriggerActive() {
    return (RobotContainer.m_xbox.getRightTriggerAxis() > 0);
  }

  public void runStingerMotor() {
    stingerMotor.set(Constants.k_stingerSpeed);
  }

  public void stopStingerMotor() {
    stingerMotor.set(Constants.k_stopStinger);
  }
}
