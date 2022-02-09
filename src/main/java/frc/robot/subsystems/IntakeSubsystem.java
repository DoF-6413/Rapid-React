package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CANSPark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
//Declaring intake motors 3 motors 2 for arms 1 for roller
    //Declare left motors
    private CANSparkMax leftLead;
    private CANSparkMax leftFollow1;
    private CANSparkMax leftFollow2;
    // Encoder left lead is used for values on the left side of the robot because they are all connected 
  private RelativeEncoder encoderLeftLead;
    private DifferentialDrive diffDrive;

    public IntakeSubsystem() { 
        //Initializes left motors in default constructor
        leftLead = new CANSparkMax(Constants.leftDeviceID[0], MotorType.kBrushless);
        leftFollow1 = new CANSparkMax(Constants.leftDeviceID[1], MotorType.kBrushless);
        leftFollow2 = new CANSparkMax(Constants.leftDeviceID[2], MotorType.kBrushless);

        encoderLeftLead = leftLead.getEncoder();
        
        //Initializes left motors in default constructor
        rightLead = new CANSparkMax(Constants.rightDeviceID[0], MotorType.kBrushless);
        rightFollow1 = new CANSparkMax(Constants.rightDeviceID[1], MotorType.kBrushless);
        rightFollow2 = new CANSparkMax(Constants.rightDeviceID[2], MotorType.kBrushless);
        encoderRightLead = rightLead.getEncoder();
        //Set left follow motors
        leftFollow1.follow(leftLead);
        leftFollow2.follow(leftLead);

        //WPI assumes that the left and right are opposite, this allows the motors to both move forward when applying positive output
        rightLead.setInverted(true);
        //Set right follow motors
        rightFollow1.follow(rightLead);
        rightFollow2.follow(rightLead);
        diffDrive = new DifferentialDrive(leftLead, rightLead);
    }

     /**
     * Sets the raw speed of the Intake using an tank style.
     * @param leftStick Value from -1.0 to 1.0 representing the left rate
     * @param rightStick  Value from -1.0 to 1.0 representing the right rate
     */
    public void setRaw(double leftStick, double rightStick) {
        System.out.println("Joystick Left input = " + leftStick);
        System.out.println("Joystick Right input = " + rightStick); 
        diffDrive.tankDrive(leftStick, rightStick);
        printEncoderStatus();
    }
    //diffDrive.tankDrive(-leftStick.getY(), -rightStick.getY());
    //parameters to pass into setRaw method

    public void printStatus(Double joystickLeftInput, Double joystickRightInput) {
        System.out.println("Joystick Left input = " + joystickLeftInput);
        System.out.println("Joystick Right input = " + joystickRightInput); 
    }
    public void printEncoderStatus() {
        SmartDashboard.putNumber("Encoder Left Lead", encoderLeftLead.getPosition());
        SmartDashboard.putNumber("Encoder Right Lead", encoderRightLead.getPosition()); 
    }

}

