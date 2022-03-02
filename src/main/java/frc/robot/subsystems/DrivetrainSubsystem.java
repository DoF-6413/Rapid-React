package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CANSPark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivetrainSubsystem extends SubsystemBase {

    //Declare left motors
    private CANSparkMax leftLead;
    private CANSparkMax leftFollow1;
    private CANSparkMax leftFollow2;
    // Encoder left lead is used for values on the left side of the robot because they are all connected 
  private RelativeEncoder encoderLeftLead;
    //Declare right motors
    private CANSparkMax rightLead;
    private CANSparkMax rightFollow1;
    private CANSparkMax rightFollow2;
    // Encoder right lead is used for values on the right side of the robot because they are all connected 
  private RelativeEncoder encoderRightLead;
    private DifferentialDrive diffDrive;

    public DrivetrainSubsystem() { 
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

        //Coverts Tics to Feet for Encoder Readout
        encoderLeftLead.setPositionConversionFactor(Constants.tick2feet);
        encoderRightLead.setPositionConversionFactor(Constants.tick2feet);
    }

     /**
     * Sets the raw speed of the drivetrain using an tank style.
     * @param leftStick Value from -1.0 to 1.0 representing the left rate
     * @param rightStick  Value from -1.0 to 1.0 representing the right rate
     */
    public void setRaw(double leftStick, double rightStick) {
        diffDrive.arcadeDrive(-rightStick, leftStick);
        printEncoderStatus();
    }
    //diffDrive.tankDrive(-leftStick.getY(), -rightStick.getY());
    //parameters to pass into setRaw method

     public void printStatus(Double joystickLeftInput, Double joystickRightInput) {
        SmartDashboard.putNumber("Joystick Left input = ", joystickLeftInput);
         SmartDashboard.putNumber("Joystick Right input = ", joystickRightInput); 
     }
    public void printEncoderStatus() {
        SmartDashboard.putNumber("Encoder Left Lead", encoderLeftLead.getPosition());
        SmartDashboard.putNumber("Encoder Right Lead", encoderRightLead.getPosition()); 
    }

}

