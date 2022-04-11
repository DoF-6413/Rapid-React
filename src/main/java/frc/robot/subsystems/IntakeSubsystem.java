package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//CANSpark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

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

  public void stopActuators() {
    intakeLeftActuator.set(0);
    intakeRightActuator.set(0);
    armPosition();
  }

  public void setActuatorUp(double speed) {
    if (toplimitSwitch.get() || 
    SmartDashboard.getNumber("Encoder Left Actuator", 0) >= -2) {
      // We are going up and top limit is tripped so stop
      intakeLeftActuator.set(0);
      intakeRightActuator.set(0);
    } else {
      // We are going up but top limit is not tripped so go at commanded speed
      intakeLeftActuator.set(speed);
      intakeRightActuator.set(-speed);
    }
    armPosition();
  }

  public void setActuatorDown(double speed) {
    if (bottomlimitSwitch.get()||
    SmartDashboard.getNumber("Encoder Left Actuator", 0) <= -38) {
      // We are going down and bottom limit is tripped so stop

      intakeLeftActuator.set(0);
      intakeRightActuator.set(0);
      down = true;
    } else {
      // We are going down but bottom limit is not tripped so go at commanded speed
      intakeLeftActuator.set(-speed);
      intakeRightActuator.set(speed);
      down = false;
    }
    armPosition();
  }

  public void setActuatorPosition(double speed){
if ((bottomlimitSwitch.get() || SmartDashboard.getNumber("Encoder Left Actuator", 0) >= -2) && speed < 0){
  intakeLeftActuator.set(0);
  intakeRightActuator.set(0);
} else if ((toplimitSwitch.get() || SmartDashboard.getNumber("Encoder Left Actuator", 0) >= -39) && speed > 0){

  intakeLeftActuator.set(0);
  intakeRightActuator.set(0);
}else {
  intakeLeftActuator.set(speed);
  intakeRightActuator.set(-speed);
}
  }
}
