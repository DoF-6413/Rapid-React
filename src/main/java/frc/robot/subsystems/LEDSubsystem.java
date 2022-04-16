// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDSubsystem extends SubsystemBase {
  private static final int kMotorPort = 1;
  private MotorController m_motor;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_motor = new Spark(kMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void allianceLED(){
    if (DriverStation.getAlliance() == Alliance.Blue){m_motor.set(.87);
    } else{ m_motor.set(.61);}
    }
  }
