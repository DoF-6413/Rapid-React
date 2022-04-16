package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CANSpark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class IntakeSubsystem extends SubsystemBase {

  // Defines Intake Motor Variables
  // Uncomment intake Acuators when we write that code
  private CANSparkMax intakeLeftActuator;
  private CANSparkMax intakeRightActuator;
  private CANSparkMax intakeSpinner;

  private RelativeEncoder leftActuatorEncoder;
  private RelativeEncoder rightActuatorEncoder;

  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  boolean down;

  // Initializing intake motors
  // Defining what each inake motor is and their ID's
  public IntakeSubsystem() {
    // Defines motors used for dropping and raising intake system (arms);uncomment
    // when we write code for Acuators
    intakeLeftActuator = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless);
    intakeRightActuator = new CANSparkMax(Constants.intakeDeviceID[1], MotorType.kBrushless);
    intakeSpinner = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless);
    
    intakeLeftActuator.setSmartCurrentLimit(10);
    intakeRightActuator.setSmartCurrentLimit(10);
    leftActuatorEncoder = intakeLeftActuator.getEncoder();
    rightActuatorEncoder = intakeLeftActuator.getEncoder();

    intakeLeftActuator.setOpenLoopRampRate(0.5);
    intakeRightActuator.setOpenLoopRampRate(0.5);
  }
  
  // Spins Intake Motor
  public void spinMotor() {
    
    intakeSpinner.set(0.5); // runs the motor at the speed set in constants% power

  }

  // Reverses Intake Motor
  public void reverseMotor() {

    intakeSpinner.set(-0.5); // runs the motor at the speed set in constants% power

  }

  // Stops Intake Motor
  public void stopMotor() {

    intakeSpinner.set(0); // stops the motor (puts it at 0% power)

  }

  public void armPosition() {
    SmartDashboard.putNumber("Encoder Left Actuator", rightActuatorEncoder.getPosition());
    SmartDashboard.putNumber("Encoder Right Actuator", leftActuatorEncoder.getPosition());
  }

  public double currentRightActuatorPosition(){
    return (SmartDashboard.getNumber("Encoder Right Actuator", leftActuatorEncoder.getPosition()));
  }

  /**
   * Gets left actuator position
   * 
   * @return Encoder Value of Left Actuator (-39 - 2)
   */
  
  public double currentLeftActuatorPosition(){
    return (SmartDashboard.getNumber("Encoder Left Actuator", rightActuatorEncoder.getPosition()));
  }
  public void stopActuators() {
    stopRightActuator();
    stopLeftActuator();
    armPosition();
  }

  public void stopLeftActuator() {
    intakeLeftActuator.set(0);
    armPosition();
  }

  public void stopRightActuator() {
    intakeRightActuator.set(0);
    armPosition();
  }

  public void setAllActuatorsUp(double speed){
    setLeftActuatorUp(speed);
    setRightActuatorUp(speed);
  }

  public void setRightActuatorUp(double speed) {
    if (toplimitSwitch.get() || 
    SmartDashboard.getNumber("Encoder Right Actuator", 0) >= -2) {
      // We are going up and top limit is tripped so stop
      intakeRightActuator.set(0);
    } else {
      // We are going up but top limit is not tripped so go at commanded speed
      intakeRightActuator.set(-speed);
    }
    armPosition();
  }

  public void setLeftActuatorUp(double speed) {
    if (toplimitSwitch.get() || 
    SmartDashboard.getNumber("Encoder Left Actuator", 0) >= -2) {
      // We are going up and top limit is tripped so stop
      intakeLeftActuator.set(0);
    } else {
      // We are going up but top limit is not tripped so go at commanded speed
      intakeLeftActuator.set(speed);
    }
    armPosition();
  }

  public void setAllActuatorsDown(double speed){
    setLeftActuatorDown(speed);
    setRightActuatorDown(speed);
  }

  /** Brings Right Actuator Down
   * @param speed -Value the actuator moves at (0-1)
   */
  public void setRightActuatorDown(double speed) {
    if (bottomlimitSwitch.get()||
    SmartDashboard.getNumber("Encoder Right Actuator", 0) <= -39) {
      // We are going down and bottom limit is tripped so stop
      intakeRightActuator.set(0);
      down = true;
    } else {
      // We are going down but bottom limit is not tripped so go at commanded speed
      intakeRightActuator.set(speed);
      down = false;
    }
    armPosition();
  }

  public void setLeftActuatorDown(double speed) {
    if (bottomlimitSwitch.get()||
    SmartDashboard.getNumber("Encoder Left Actuator", 0) <= -39) {
      // We are going down and bottom limit is tripped so stop

      intakeLeftActuator.set(0);
      down = true;
    } else {
      // We are going down but bottom limit is not tripped so go at commanded speed
      intakeLeftActuator.set(-speed);
      down = false;
    }
    armPosition();
  }

 public void climbingCurrentLimit(){
  intakeLeftActuator.setSmartCurrentLimit(20);
  intakeRightActuator.setSmartCurrentLimit(20);
 }
}
