// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        // Sets Intake Forward (To Shooter) Speed (Currently 50%)
        final public static double intakeSpeed = 0.5;
        
    final public static double reverseIntakeSpeed = -0.5;
    final public static double slowSpeed = 0.50;

    // FACTS
    final public static double wheelDiameter = 4.;
    final public static double tick2feet = 1.0 / 42. * wheelDiameter * 6.11 * Math.PI / 12.;
    // final public static double FalconTurnsPerRotation = 3600;

    // Sets indexer speed (Currently 20%)
    final public static double indexerSpeed = .60;
// switched to positive march 11
    // Falcon 500 RPM Equation
    final public static double velocityToRPM(double velocity) {
        return velocity * 600 / 2048;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~SHOOTER
    // CONSTANTS~~~~~~~~~~~~~~~~~~~~~~~~~
    final public static double kP = 0.5;
    final public static double kI = 15;
    final public static double kD = 45;

    final public static double kShooterTolerance = 1.0;
    final public static double kShooterDerivativeTolerance = 0.01;
    // auto values
    // units in feet
    final public static double moveDistanceFromTarmac = 5;

    // shooter run times seconds
    final public static double waitTimeForShooter = 15; 
    final public static double oneSecond = 1;
    final public static double twoSeconds = 2;
    final public static double twoAndHalfSeconds = 2.5;
    final public static double threeSeconds = 3;
    final public static double fourSeconds = 4;

    // indexer wait time to optimize shooter control in teleop
    final public static double indexerWaitTime = 0.25;
    //MATH
    final public static double wheelToWheel = 26;
    final public static double turningRadius = wheelToWheel/2;
    final public static double turningSomething = wheelToWheel * 0.01745329;
    final public static double circOfWheelToWheel = wheelToWheel/12 /2 * 2 * Math.PI;

    // final public static double oneEightyTurn = circOfWheelToWheel/2;
    // final public static double ninetyTurn = circOfWheelToWheel/4;
    // final public static double fortyFiveTurn = circOfWheelToWheel/8;

    //Shooter Speeds
    final public static double lowerHubSpeed = 2300;
    final public static double upperHubSpeed = 4000;



    final public static boolean K_GYRO_REVERSED = false;

    /* Drivetrain PID */
  public static final double K_CHASSIS_TURN_P = 0.8;
  public static final double K_CHASSIS_TURN_I = 0;
  public static final double K_CHASSIS_TURN_D = 0;
  public static final double K_TURN_TOLERANCE_DEG = 2;
  public static final double K_TURN_RATE_TOLERANCE_DEG_PER_SEC = 2;
}
