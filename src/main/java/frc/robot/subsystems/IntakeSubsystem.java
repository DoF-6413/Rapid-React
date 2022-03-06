package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CANSpark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  //Defines Intake Motor Variables
  //Uncomment intake Acuators when we write that code
    private CANSparkMax intakeLeftActuator;
    private CANSparkMax intakeRightActuator;
    private CANSparkMax intakeSpinner;



    //Initializing intake motors
    //Defining what each inake motor is and their ID's
    public IntakeSubsystem() {
      //Defines motors used for dropping and raising intake system (arms);uncomment when we write code for Acuators
      //intakeActuatorLead = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless);   
      //intakeActuatorFollow = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless);
      intakeSpinner = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless);
      intakeLeftActuator = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless) ;
      intakeRightActuator = new CANSparkMax(Constants.intakeDeviceID[1], MotorType.kBrushless) ;
    }

//Spins Intake Motor
  public void spinMotor(){

    intakeSpinner.set(Constants.intakeSpeed); // runs the motor at the speed set in constants% power

}
//Reverses Intake Motor
public void reverseMotor(){

  intakeSpinner.set(Constants.reverseIntakeSpeed); // runs the motor at the speed set in constants% power

}
//Stops Intake Motor
public void stopMotor(){

  intakeSpinner.set(0) ; //stops the motor (puts it at 0% power)

}

public void setOrigin(){
  intakeLeftActuator.set(.10);
  intakeRightActuator.set(-0.10);
}

public void goDown(){
  intakeLeftActuator.set(-0.10);
  intakeRightActuator.set(0.10);
}

public void stopActuators(){
  intakeLeftActuator.set(0);
  intakeRightActuator.set(0);
}

// TODO: Create a function that moves the aucuator 90 degrees to drop intake system
}
