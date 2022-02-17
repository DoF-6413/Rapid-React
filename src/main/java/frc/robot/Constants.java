// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // defining joystick ports
    final public static int initialJoystickPort = 0;
    final public static int secondaryJoystickPort = 1;
    // defining axis
    final public static int joystickAxis = 1;

    final public static int[] leftDeviceID = new int[]{1, 2, 3}; //CAN ID configured using Spark MAX Client
    final public static int[] rightDeviceID = new int[]{4, 5, 6}; 

    final public static int[] intakeDeviceID = new int []{8, 9, 10}; // Intake Motors, Number 8 and 10 for Arm, 9 for IntakeSpin

    //encoder values
    final public static int wheelDiameter = 4; 
    final public static double tick2feet = 1.0/42. * 4. * 6.11 * Math.PI / 12.;

    //Assigned to Left Joystick Trigger
    final public static int intakeTrigger = 1;

    //Sets Intake Speed (Currently 50%)
    final public static double intakeSpeed = 0.5;

}
