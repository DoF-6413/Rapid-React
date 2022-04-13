// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

/**
 * Command that will move the the robot a specific distance (in units of feet)
 */
public class MoveCommand extends CommandBase {
    public int toDistance;

    public boolean goesForward;

    private final DrivetrainSubsystem m_DrivetrainSubsystem;

    /**
     * Creates a new MoveCommand.
     *
     * @param drivetrainSubsystem - The drivetrain to use
     * @param Distance - The distance to move (in feet)
     * @param Forwards - true if the distance is forward, false if backwards
     */
    public MoveCommand(DrivetrainSubsystem drivetrainSubsystem, Integer Distance, boolean Forwards) {
        m_DrivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_DrivetrainSubsystem);
        toDistance = Distance;
        goesForward = Forwards;
        }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_DrivetrainSubsystem.resetEncoderValue();
        }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (goesForward) {
            if (m_DrivetrainSubsystem.getAvgEncoderDistance() < toDistance)
                m_DrivetrainSubsystem.setRaw(-0.5,  Constants.kOff );
            else {
                m_DrivetrainSubsystem.setRaw( Constants.kOff, Constants.kOff );
            }
        } else if (!goesForward) {
            if (m_DrivetrainSubsystem.getAvgEncoderDistance() > toDistance)
                m_DrivetrainSubsystem.setRaw(0.5, Constants.kOff );
        } else {
            m_DrivetrainSubsystem.setRaw( Constants.kOff, Constants.kOff);
        }
        }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_DrivetrainSubsystem.setRaw( Constants.kOff, Constants.kOff );
        }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return goesForward ?  (m_DrivetrainSubsystem.getAvgEncoderDistance() >= toDistance) : (m_DrivetrainSubsystem.getAvgEncoderDistance() <= toDistance);
        }
}