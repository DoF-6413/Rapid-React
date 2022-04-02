// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  AHRS gyro;
  /** Creates a new ExampleSubsystem. */
  public GyroSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP); 
    
  }
  
  
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Raw Gyro Y", gyro.getRawGyroY());
    SmartDashboard.putNumber("ROLL", gyro.getRoll());
    SmartDashboard.putNumber("Raw Gyro X", gyro.getRawGyroX());
    SmartDashboard.putNumber("PITCH", gyro.getPitch());
    SmartDashboard.putNumber("Raw Gyro Z", gyro.getRawGyroZ());
    SmartDashboard.putNumber("YAW", gyro.getYaw());
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetYaw () {
    gyro.reset();
    
  }
}
