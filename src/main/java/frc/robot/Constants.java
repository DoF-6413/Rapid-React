// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    //Ports

        // defining xbox port
        final public static int driverXboxPort = 0;
        final public static int auxXboxPort = 1;

    // defining axis
    final public static int joystickYAxis = 1;
    final public static int joystickXAxis = 0;

    // Defining CANIDs
        //Drivetrain CAN IDs
        final public static int[] leftDeviceID = new int[] { 1, 2, 3 }; // CAN ID configured using Spark MAX Client
        final public static int[] rightDeviceID = new int[] { 4, 5, 6 };

        //Intake CAN IDs
        final public static int[] intakeDeviceID = new int[] { 8, 9, 7 }; 
                                                                       
        //Shooter CAN IDs
        final public static int[] shooterID = new int[] { 10, 11 };

        //Indexer CAN ID
        final public static int IndexerID = 12;
        
        //Climber CAN ID
        final public static int[] ClimberID = new int[] {13, 14, 15};
        final public static int k_topLiftMotor = 0;
        final public static int k_bottomLiftMotor = 1;
        final public static int k_stingerMotor = 2;

        //Limit Switch (Digial Input) IDs 
        final public static int[] limitSwitchID = new int[] {0, 1, 2};
    //Intake Constants
        //Intake Speeds
            // Sets Intake Forward (To Shooter) Speed (Currently 50%)
            final public static double intakeSpeed = 0.5;
            // Sets Intake Backwards (Away from Shooter) Speed (Currently 50%)
            final public static double reverseIntakeSpeed = -0.5;
            // Sets Intake Actuator Up Speed
            final public static double actuatorsSpeed = 0.50;
            //Intake Current Limit
            final public static int k_intakeCurrentLimit = 10;
            //Intake Ramp Rate
            final public static double k_intakeRampRate = 0.5;
    
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
        final public static double k_climberMaxUp = 0.6;
        //Sets Auto Climber Speed Down
        final public static double k_climberMaxDown = -0.6;
        //Sets Manual Climber Speed Up
        final public static double k_climberMinUp = 0.3;
        //Sets Manual Climber Speed Down
        final public static double k_climberMinDown = -0.3;
        //Sets Climber Highest Value Current Limit
        final public static int k_climberHighestCurrentLimit = 60;
        //Sets Climber Lowest Value Current Limit
        final public static int k_climberLowestCurrentLimit = 10;
        //Climber Top Position
        final public static double k_climberTop = 21.5; // TODO: Update Value
        //Climber Bottom Position
        final public static int k_climberBottom = 0; // TODO: Update Value
        //Stinger speed
        final public static double k_stingerSpeed = 1;



    //Shooter Contants
        //Shooter PID
        final public static double kP = 0.5;
        final public static double kI = 15;
        final public static double kD = 45;
        final public static double k_shooterTolerance = 50;
        final public static double kShooterDerivativeTolerance = 0.01;
        //Shooter Speeds
        final public static double lowerHubSpeed = 1500;
        final public static double upperHubSpeed = 2800;
        
    //Gyro Contants
        final public static boolean K_GYRO_REVERSED = false;
    
    //Drivetrain Constants
        //Drivetrain PID (Only used for turning in autonomous)
        public static final double K_CHASSIS_TURN_P = 2;
        public static final double K_CHASSIS_TURN_I = 0.0;
        public static final double K_CHASSIS_TURN_D = 0;
        public static final double K_TURN_TOLERANCE_DEG = 20;
        public static final double K_TURN_RATE_TOLERANCE_DEG_PER_SEC = 2;
        //Turn Scaling
        public static final double turningScale = 0.5;
        //Drivetrain Auto Forwards
        public static final double k_autoDriveForwardSpeed = 0.5;
        //Drivetrain Auto Backwards
        public static final double k_autoDriveBackwardsSpeed = -0.5;
        //Drivetrain Start Position
        public static final double k_drivetrainStart = 0;
        //Drivetrain 3 Ball First Turn Angle
        public static final double k_firstTurnAuto = 90;
        //Drivetrain 3 Ball First Turn Angle
        public static final double k_secondTurnAuto = -90;
        //Drivetrain 3 ball auto distance
        public static final int k_moveThreeBall = -6;

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
            //stop any motor speed
            final public static double k_stopMotor = 0;
        //MATH FOR TURNING
            //Wheel to Wheel Distance (Mid Left Side to Mid Right Side)
            final public static double wheelToWheel = 26;
            final public static double turningRadius = wheelToWheel/2;
            final public static double turningSomething = wheelToWheel * 0.01745329;
            final public static double circOfWheelToWheel = wheelToWheel/12 /2 * 2 * Math.PI;

        //Limelight Min and Max Sweet Spot Values
        public static final double k_limelightYMax = 11;
        public static final double k_limelightYMin = 7;

        public static final double k_limelightXMax = 3;
        public static final double k_limelightXMin = -3;
        // Default Limelight Return Values when Recieving no Input
        public static final double k_defaultReturn = 0;
}
