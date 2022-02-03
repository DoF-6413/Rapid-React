package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public Drivetrain() { 
    
    }
    public void printStatus(double leftStickValues)  
    {
        System.out.println(leftStickValues); 
    
    }
}

