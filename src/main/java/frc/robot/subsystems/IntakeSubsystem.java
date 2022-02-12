package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CANSPark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax actuator1;
    private CANSparkMax actuator2;
    private CANSparkMax spinner;






    //Initializing intake motors
    public IntakeSubsystem() {
      actuator1 = new CANSparkMax(Constants.actuator1Port, MotorType.kBrushless);   
      actuator2 = new CANSparkMax(Constants.actuator2Port, MotorType.kBrushless);
      spinner = new CANSparkMax(Constants.spinner, MotorType.kBrushless);









    }


    public void print() {
      System.out.println("Button Pressed Input");


    }



  }

