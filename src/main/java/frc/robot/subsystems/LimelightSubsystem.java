// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
  }

  // current LedMode, 0= on, 1 = off
  private int CurMode = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void periodic() {
    double ty = this.getTy();
    double tx = this.getTx();
    boolean tv = this.hasTarget();
    SmartDashboard.putNumber("LLx", tx);
    SmartDashboard.putNumber("LLy", ty);
    SmartDashboard.putBoolean("LLv", tv);
    // This method will be called once per scheduler run
  }

  public double getTx() {
    return table.getEntry("tx").getDouble(Constants.k_defaultReturn);
  }

  public double getTy() {
    return table.getEntry("ty").getDouble(Constants.k_defaultReturn);
  }

  public boolean hasTarget() {

    return ( table.getEntry("tv").getDouble(Constants.k_defaultReturn) != 0.0f );
  
  }

  public void ledOFF() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void ledON() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void toggleLED() {
    if (CurMode == 0) {
      ledON();
      CurMode = 1;
    } else {
      ledOFF();
      CurMode = 0;
    }
  }

  public void limelightReadong() {
    System.out.println("MOVING TO" + hasTarget());
  }
}
