package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CANSpark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {

  //Defines Intake Motor Variables
  //Uncomment intake Acuators when we write that code
    private CANSparkMax intakeLeftActuator;
    private CANSparkMax intakeRightActuator;
    private CANSparkMax intakeSpinner;

  //Defines Encoders for Actuator Motors
  private RelativeEncoder leftActuatorEncoder;
  private RelativeEncoder rightActuatorEncoder;

    //Initializing intake motors
    //Defining what each inake motor is and their ID's
    public IntakeSubsystem() {
      //Defines motors used for dropping and raising intake system (arms);uncomment when we write code for Acuators
      //intakeActuatorLead = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless);   
      //intakeActuatorFollow = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless);
      intakeSpinner = new CANSparkMax(Constants.intakeDeviceID[1], MotorType.kBrushless);
      intakeLeftActuator = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless) ;
      intakeRightActuator = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless) ;

      //Sets Intake Acuator Encoders to read Values from the Speed Controllers
      leftActuatorEncoder = intakeLeftActuator.getEncoder();
      rightActuatorEncoder = intakeRightActuator.getEncoder();

      //GO CRAZY (Bannanas)
      rightActuatorEncoder.getPosition();
      leftActuatorEncoder.getPosition();

      //Displays Actuator Encoder Positions 
      SmartDashboard.putNumber("Left Actuator Encoder", leftActuatorEncoder.getPosition());
      SmartDashboard.putNumber("Right Actuator Encoder", rightActuatorEncoder.getPosition());
    }

//Spins Intake Motor
  public void spinMotor(){

    intakeSpinner.set(Constants.intakeSpeed); // runs the motor at the speed set in constants% power

}
//Stops Intake Motor
public void stopMotor(){

  intakeSpinner.set(0) ; //stops the motor (puts it at 0% power)

}

// TODO: Create a function that moves the aucuator 90 degrees to drop intake system
}
