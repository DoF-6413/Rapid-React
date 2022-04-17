// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}
  
  // current LedMode, 0= on, 1 = off
  private int CurMode = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getTx ()
  {
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0);
    SmartDashboard.putNumber("LimelightX", x);
    return x;
  }

  public double getTy ()
  {
    NetworkTableEntry ty = table.getEntry("ty"); 
    double y = ty.getDouble(0);
    SmartDashboard.putNumber("LimelightY", y);
     return y;
  }

  public boolean hasTarget ()
  {
    boolean targetSeen = (table.getEntry("tv").getDouble(0) == 0);
    SmartDashboard.putBoolean("LimelightY", targetSeen);
     return targetSeen;
  }

  public void ledOFF()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void ledON()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void toggleLED()
  {
    if (CurMode == 0)
    { 
      ledON();
      CurMode=1;
    } else {
      ledOFF();
      CurMode =0;
    }
  }

  public void limelightReadong (){
    System.out.println("MOVING TO" + hasTarget());
  }
}
