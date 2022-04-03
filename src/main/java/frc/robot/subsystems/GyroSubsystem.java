// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  AHRS gyro;
  /** Creates a new SmartDashboardSubsystem. */
  public GyroSubsystem() {

    gyro = new AHRS(SPI.Port.kMXP);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro X", gyro.getRawGyroX());
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Gyro Y", gyro.getRawGyroY());
    SmartDashboard.putNumber("Roll", gyro.getRoll());
    SmartDashboard.putNumber("Gyro Z", gyro.getRawGyroZ());
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Yaw", gyro.getAngle());
  }

  public void GyroReadings(){
    
  }

  public void resetYaw () {
    gyro.reset();
  }

  /**
   * Get heading of the robot (no domain).
   * @return the angle of the gyro in degrees.
   */
  public double getAngle (){
    return gyro.getAngle();
  }
}
