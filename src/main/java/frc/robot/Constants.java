// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Constants {
    //Ports

        // defining joystick ports
        final public static int initialJoystickPort = 0;
        final public static int secondaryJoystickPort = 1;

        // defining xbox port
        final public static int xboxPort = 2;

    // defining axis
    final public static int joystickYAxis = 1;
    final public static int joystickXAxis = 0;

    // Defining CANIDs
        //Drivetrain CAN IDs
        final public static int[] leftDeviceID = new int[] { 1, 2, 3 }; // CAN ID configured using Spark MAX Client
        final public static int[] rightDeviceID = new int[] { 4, 5, 6 };

        //Intake CAN IDs
        final public static int[] intakeDeviceID = new int[] { 8, 9, 10 }; 
                                                                       
        //Shooter CAN IDs
        final public static int[] shooterID = new int[] { 7, 11 };

        //Indexer CAN ID
        final public static int IndexerID = 12;
        
        //Climber CAN ID
        final public static int ClimberID = 13;


    //Intake Constants
        //Intake Speeds
            // Sets Intake Forward (To Shooter) Speed (Currently 50%)
            final public static double intakeSpeed = 0.5;
            // Sets Intake Backwards (Away from Shooter) Speed (Currently 50%)
            final public static double reverseIntakeSpeed = -0.5;
            // Sets Intake Actuator Up Speed
            final public static double actuatorsSpeed = 0.50;
    
        //Intake PID (Only Used When Climbing)
        public static final double K_IntakeActuator_P = 0;
        public static final double K_IntakeActuator_I = 0;
        public static final double K_IntakeActuator_D = 0;
        public static final double K_TiltToleranceDeg = 2;
        public static final double K_TiltRateToleranceDegPerSec = 2;
      
    //Indexer Constants
        // Sets indexer speed (Currently 60%)
        final public static double indexerSpeed = .60;
        // indexer wait time to optimize shooter control in teleop
        final public static double indexerWaitTime = 0.25;

    //Climber Constants
        //Sets Auto Climber Speed Up
        final public static double climberSpeedUp = 0.6;
        //Sets Auto Climber Speed Down
        final public static double climberSpeedDown = -0.6;
        //Sets Manual Climber Speed Up
        final public static double manualClimberSpeedUp = 0.5;
        //Sets Manual Climber Speed Down
        final public static double manualClimberSpeedDown = -0.5;

    //Shooter Contants
        //Shooter PID
        final public static double kP = 0.5;
        final public static double kI = 15;
        final public static double kD = 45;
        final public static double kShooterTolerance = 1.0;
        final public static double kShooterDerivativeTolerance = 0.01;
        //Shooter Speeds
        final public static double lowerHubSpeed = 2300;
        final public static double upperHubSpeed = 4000;
        
    //Gyro Contants
        final public static boolean K_GYRO_REVERSED = false;
    
    //Drivetrain Constants
        //Drivetrain PID (Only used for turning in autonomous)
        public static final double K_CHASSIS_TURN_P = 0.8;
        public static final double K_CHASSIS_TURN_I = 0.55;
        public static final double K_CHASSIS_TURN_D = 0;
        public static final double K_TURN_TOLERANCE_DEG = 2;
        public static final double K_TURN_RATE_TOLERANCE_DEG_PER_SEC = 2;
        //Turn Scaling
        public static final double turningScale = 0.5;

    //Auto Values
        // shooter run times in seconds
        final public static double waitTimeForShooter = 15; 
        final public static double oneSecond = 1;
        final public static double twoSeconds = 2;
            final public static double twoAndHalfSeconds = 2.5;
            final public static double threeSeconds = 3;
            final public static double fourSeconds = 4;
        
        // FACTS
            //Drivetrain Wheel Diameter
            final public static double wheelDiameter = 4;
            //Drivetrain Distance in Feet Calculation
            final public static double tick2feet = 1.0 / 42. * wheelDiameter * 6.11 * Math.PI / 12.;
            // Falcon 500 RPM Equation
            final public static double velocityToRPM(double velocity) {
                return velocity * 600 / 2048;
            }
        //MATH FOR TURNING
            //Wheel to Wheel Distance (Mid Left Side to Mid Right Side)
            final public static double wheelToWheel = 26;
            final public static double turningRadius = wheelToWheel/2;
            final public static double turningSomething = wheelToWheel * 0.01745329;
            final public static double circOfWheelToWheel = wheelToWheel/12 /2 * 2 * Math.PI;

        //Limelight Min and Max Sweet Spot Values
        public static final double limelightYMax = 11;
        public static final double limelightYMin = 8;

        public static final double limelightXMax = 3;
        public static final double limelightXMin = -3;
}
