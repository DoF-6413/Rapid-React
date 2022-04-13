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

    // Initializing intake motors
    // Defining what each inake motor is and their ID's
    public IntakeSubsystem() {
        // Defines motors used for dropping and raising intake system (arms)
        intakeLeftActuator = new CANSparkMax(Constants.intakeDeviceID[0], MotorType.kBrushless);
        intakeRightActuator = new CANSparkMax(Constants.intakeDeviceID[1], MotorType.kBrushless);
        intakeSpinner = new CANSparkMax(Constants.intakeDeviceID[2], MotorType.kBrushless);

        // We need to current limit the motors so we don't brown out (esp. when turning)
        intakeLeftActuator.setSmartCurrentLimit( Constants.k10Percent );
        intakeRightActuator.setSmartCurrentLimit( Constants.k10Percent );

        leftActuatorEncoder = intakeLeftActuator.getEncoder();
        rightActuatorEncoder = intakeLeftActuator.getEncoder();
        }


    /**
     * Spins the intake motor
     */
    public void spinMotor() {
        intakeSpinner.set( Constants.intakeSpeed );
        }


    /**
     * Reverses the intake motor
     */
    public void reverseMotor() {
        intakeSpinner.set( Constants.reverseIntakeSpeed );
        }


    /**
     * Stop the intake motor
     */
    public void stopMotor() {
        intakeSpinner.set( Constants.kOff ); // stops the motor (puts it at 0% power)
        }


    /**
     * Update the intake arm position on the SmartDashboard
     */
    public void armPosition() {
        SmartDashboard.putNumber("Encoder Left Actuator", rightActuatorEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Right Actuator", leftActuatorEncoder.getPosition());
        }


    /**
     * Stop the actuator motors
     */
    public void stopActuators() {
        intakeLeftActuator.set(0);
        intakeRightActuator.set(0);
        armPosition();
        }


    /**
     * Start moving the arms up at the given speed
     * 
     * @param speed - The speed to move the arms up at
     */
    public void setActuatorUp(double speed) {
        if ( toplimitSwitch.get() || SmartDashboard.getNumber("Encoder Left Actuator", 0) >= -1) {
            // We want to go up but we the top limit switch tripped so stop the motors since we are fully "up"
            intakeLeftActuator.set( Constants.kOff );
            intakeRightActuator.set( Constants.kOff );
            }
        else {
            // We are not fully up so turn the motors on for the commanded speed
            intakeLeftActuator.set(speed);
            intakeRightActuator.set(-speed);
            }

        armPosition();
        }


    /**
     * Start moving the arms down at the given speed
     * 
     * @param speed - The speed to move the arms up at
     */
    public void setActuatorDown(double speed) {
        if (bottomlimitSwitch.get()) { // || SmartDashboard.getNumber("Encoder Left Actuator", 0) <= -38) 
            // We want to go down but the bottom limit switch tripped so stop the motors sincde we are fully "down"

            intakeLeftActuator.set( Constants.kOff );
            intakeRightActuator.set( Constants.kOff );
        } 
        else {
            // We are not fully down so turn the motors on for the commanded speed
            intakeLeftActuator.set( -speed );
            intakeRightActuator.set( speed );
        }

        armPosition();
        }

    // TODO: Create a function that moves the acuator 90 degrees to drop intake system (Huh??)

    /**
     * Are the acutator arms down?  We decide this by relying on the bottom limit switch. 
     * 
     * @return true if they are, false otherwise
     */
    public boolean isDown(){
        return bottomlimitSwitch.get();
        }


    /**
     * Are the acutator arms up?  We decide this by relying on the top limit switch. 
     * 
     * @return true if they are, false otherwise
     */
    public boolean isUp(){
        return toplimitSwitch.get();
        }
}
