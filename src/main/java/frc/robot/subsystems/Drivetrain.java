package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CANSPark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Drivetrain extends SubsystemBase {

    //Declare left motors
    private CANSparkMax leftLead;
    private CANSparkMax leftFollow1;
    private CANSparkMax leftFollow2;
  
    //Declare right motors
    private CANSparkMax rightLead;
    private CANSparkMax rightFollow1;
    private CANSparkMax rightFollow2;
  
    private DifferentialDrive diffDrive;

    public Drivetrain() { 
        //Initializes left motors in default constructor
        leftLead = new CANSparkMax(Constants.leftDeviceID[0], MotorType.kBrushless);
        leftFollow1 = new CANSparkMax(Constants.leftDeviceID[1], MotorType.kBrushless);
        leftFollow2 = new CANSparkMax(Constants.leftDeviceID[2], MotorType.kBrushless);
        
        //Initializes left motors in default constructor
        rightLead = new CANSparkMax(Constants.leftDeviceID[0], MotorType.kBrushless);
        rightFollow1 = new CANSparkMax(Constants.leftDeviceID[1], MotorType.kBrushless);
        rightFollow2 = new CANSparkMax(Constants.leftDeviceID[2], MotorType.kBrushless);

        //Set left follow motors
        leftFollow1.follow(leftLead);
        leftFollow2.follow(leftLead);

        //WPI assumes that the left and right are opposite, this allows the motors to both move forward when applying positive output
        //rightLead.setInverted(false);
        //Set right follow motors
        rightFollow1.follow(rightLead);
        rightFollow2.follow(rightLead);

        diffDrive = new DifferentialDrive(leftLead, rightLead);
    }

     /**
     * Sets the raw speed of the drivetrain using an tank style.
     * @param leftStick Value from -1.0 to 1.0 representing the left rate
     * @param rightStick  Value from -1.0 to 1.0 representing the right rate
     */
    public void setRaw(double leftStick, double rightStick) {

        diffDrive.tankDrive(-leftStick, -rightStick);
    }
    //diffDrive.tankDrive(-leftStick.getY(), -rightStick.getY());
    //parameters to pass into setRaw method

    public void printStatus() {
        System.out.println("tanisha said pineapples are crazy"); 

    }

}

