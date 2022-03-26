// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  private double Position;
  TalonFX climberMotor = new TalonFX(Constants.ClimberID);
  public ClimberSubsystem() {
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.statorCurrLimit.enable = false;
    config.statorCurrLimit.currentLimit = 30;
    climberMotor.configAllSettings(config); 
  }
  public void goUp() {
    //climberMotor.set(TalonFXControlMode.PercentOutput, 0.50); // runs the motor at 0% power
    if (Position >= 40){
      climberMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else {
      climberMotor.set(TalonFXControlMode.PercentOutput, 0.80);
    }
    climberPosition();
  }

  public void goDown() {
    //climberMotor.set(TalonFXControlMode.PercentOutput, -0.5); // runs the motor at 0% power
    if (Position <= 0){
      climberMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else {
      climberMotor.set(TalonFXControlMode.PercentOutput, -0.50);
    }
    climberPosition();
  }

  public void goDownManual(){
    climberMotor.set(TalonFXControlMode.PercentOutput, -0.20);
  }

  public void goUpManual(){
    climberMotor.set(TalonFXControlMode.PercentOutput, 0.20);
  }

  public void stop() {
    climberMotor.set(TalonFXControlMode.PercentOutput, 0); // runs the motor at 0% power
    climberPosition();
  }

  public void climberPosition() {
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getSelectedSensorPosition()/6380);   
    SmartDashboard.putNumber("Climber Current", climberMotor.getStatorCurrent());   
    Position = climberMotor.getSelectedSensorPosition()/6380;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
