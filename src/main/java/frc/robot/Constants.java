// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // defining joystick ports
    final public static int initialJoystickPort = 0;
    final public static int secondaryJoystickPort = 1;
    //defining xbox port
    final public static int xboxPort = 2;
    // defining axis
    final public static int joystickYAxis = 1;
    final public static int joystickXAxis = 0;

    final public static int[] leftDeviceID = new int[] { 1, 2, 3 }; // CAN ID configured using Spark MAX Client
    final public static int[] rightDeviceID = new int[] { 4, 5, 6 };

    final public static int[] intakeDeviceID = new int []{8, 9, 10}; // Intake Motors, Number 8 and 10 for Arm, 9 for IntakeSpin

    final public static int[] shooterID = new int[] { 7, 11 };
    final public static int IndexerID = 12;
    //Assigned to Left Joystick Trigger
    final public static int intakeTrigger = 1;

    //Sets Intake Speed (Currently 50%)
    final public static double intakeSpeed = 0.5;
    final public static double reverseIntakeSpeed = -0.5;
    final public static double slowSpeed = 0.1;

    //Xbox Buttons
    final public static int xboxA = 1;
    final public static int xboxB = 2;
    final public static int xboxX = 3;
    final public static int xboxY = 4;
    final public static double wheelDiameter = 4.; 
    final public static double tick2feet = 1.0/42. * wheelDiameter * 6.11 * Math.PI / 12.;


    // CAn ID for Shooter

    final public static int shooterButton = 1;

    // final public static double FalconTurnsPerRotation = 3600;

    // Sets shooter speed (Currently 90%)
    final public static double shooterSpeed = .90;

    // Sets indexer speed (Currently 50%)
    final public static double indexerSpeed = -0.20;

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
}
