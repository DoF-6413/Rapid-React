package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
//import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

//CANSPark imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class DrivetrainSubsystem extends SubsystemBase {

    // Declare left motors
    private CANSparkMax leftLead;
    private CANSparkMax leftFollow1;
    private CANSparkMax leftFollow2;
    // Encoder left lead is used for values on the left side of the robot because
    // they are all connected
    // Declare right motors
    private CANSparkMax rightLead;
    private CANSparkMax rightFollow1;
    private CANSparkMax rightFollow2;
    // Encoder right lead is used for values on the right side of the robot because
    // they are all connected
    
    private RelativeEncoder encoderLeftLead;
    private RelativeEncoder encoderRightLead;
    
    double RightStick;
    double LeftStick;
    
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.wheelToWheel));
    private DifferentialDrive diffDrive;
    private final DifferentialDriveOdometry  m_odometry = new DifferentialDriveOdometry(getHeading());
    Pose2d m_pose;
    
    private final GyroSubsystem m_gyroSubsystem = new GyroSubsystem();
    
    public DrivetrainSubsystem() {
        // Initializes left motors in default constructor
        leftLead = new CANSparkMax(Constants.leftDeviceID[0], MotorType.kBrushless);
        leftFollow1 = new CANSparkMax(Constants.leftDeviceID[1], MotorType.kBrushless);
        leftFollow2 = new CANSparkMax(Constants.leftDeviceID[2], MotorType.kBrushless);

        encoderLeftLead = leftLead.getEncoder();

        // Initializes left motors in default constructor
        rightLead = new CANSparkMax(Constants.rightDeviceID[0], MotorType.kBrushless);
        rightFollow1 = new CANSparkMax(Constants.rightDeviceID[1], MotorType.kBrushless);
        rightFollow2 = new CANSparkMax(Constants.rightDeviceID[2], MotorType.kBrushless);

        encoderRightLead = rightLead.getEncoder();
        // Set left follow motors
        leftFollow1.follow(leftLead);
        leftFollow2.follow(leftLead);

        // WPI assumes that the left and right are opposite, this allows the motors to
        // both move forward when applying positive output
        rightLead.setInverted(true);
        // Set right follow motors
        rightFollow1.follow(rightLead);
        rightFollow2.follow(rightLead);
        diffDrive = new DifferentialDrive(leftLead, rightLead);

        // Coverts Tics to Feet for Encoder Readout
        encoderLeftLead.setPositionConversionFactor(Constants.tick2feet);
        encoderRightLead.setPositionConversionFactor(Constants.tick2feet);

        leftLead.setIdleMode(CANSparkMax.IdleMode.kBrake); // setIdleMode​(CANSparkMax.IdleMode.kBrake)
        leftFollow1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollow2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollow1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollow2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // leftLead.setSmartCurrentLimit(10);
        // rightLead.setSmartCurrentLimit(10);

        // TO DO: test ramping with drivetrain (Need driveteam to see how they like it) 
        //might need more custimization (Slower when stoping, faster when starting)
        //leftLead.setOpenLoopRampRate(0.5);
        //rightLead.setOpenLoopRampRate(0.5);
        
       
        
        
    }
    
    /**
     * Sets the raw speed of the drivetrain using an tank style.
     * 
     * @param leftStick  Value from -1.0 to 1.0 representing the left rate
     * @param rightStick Value from -1.0 to 1.0 representing the right rate
     */
    public void setRaw(double rightStick, double leftStick) {

        diffDrive.arcadeDrive(-(rightStick), (leftStick));
        printEncoderStatus();
    }
    

    /*
     * to switch to tank drive uncomment this and place inside of set raw
     * diffDrive.tankDrive(-leftStick.getY(), -rightStick.getY());
     * parameters to pass into setRaw method
     */

    public void printStatus(Double joystickLeftInput, Double joystickRightInput) {
        SmartDashboard.putNumber("Joystick Left input = ", joystickLeftInput);
        SmartDashboard.putNumber("Joystick Right input = ", joystickRightInput);
    }

    public void printEncoderStatus() {
        SmartDashboard.putNumber("Encoder Left Lead", encoderLeftLead.getPosition());
        SmartDashboard.putNumber("Encoder Right Lead", encoderRightLead.getPosition());
    }

    public double getAvgEncocderDistance() {
        double averageEncoderDistance = (encoderLeftLead.getPosition() + encoderRightLead.getPosition())/2.0 ;
        System.out.println ("averageEncoderDistance" + averageEncoderDistance);
        return averageEncoderDistance;
    }

    public double LeftEncoderDistance(){
        double leftEncoderDistance = (encoderLeftLead.getPosition());
        System.out.println ("leftEncoderDistance" + leftEncoderDistance);
        return leftEncoderDistance;
    }

    public double RightEncoderDistance(){
        double rightEncoderDistance = (encoderRightLead.getPosition());
        System.out.println ("rightEncoderDistance" + rightEncoderDistance);
        return rightEncoderDistance;
    }

    public void resetEncoderValue() {
        encoderLeftLead.setPosition(0);
        encoderRightLead.setPosition(0);
    }

    // public double LeftEncoderVelocity(){
    //     double leftEncoderVelocity = (encoderLeftLead.getVelocity());
    //     // System.out.println ("leftEncoderDistance" + leftEncoderDistance);
    //     return leftEncoderVelocity;
    // }

    // public double RightEncoderVelocity(){
    //     double rightEncoderVelocity = (encoderRightLead.getVelocity());
    //     // System.out.println ("rightEncoderDistance" + rightEncoderDistance);
    //     return rightEncoderVelocity;
    // }
    
      /**
       * Get gyro heading between -180 to 180.
       * Uses Math.IEEEremainder to get range of -180 to 180 --> dividend - (divisor * Math.Round(dividend / divisor)).
       * @return the robot's heading in degrees.
       */
    //   public double getHeading()
    //   {
    //     return Math.IEEEremainder(m_gyroSubsystem.getAngle(), 360) * (Constants.K_GYRO_REVERSED ? -1.0 : 1.0);
    //   }
    
        public Rotation2d getHeading()
        {
            //questionable....
            return Rotation2d.fromDegrees(-m_gyroSubsystem.getAngle());
        }

        // public DifferentialDriveWheelSpeeds getSpeed()
        // {
        //     return new DifferentialDriveWheelSpeeds(
        //         LeftEncoderVelocity(), RightEncoderVelocity());
        // }

      public void current(){
        SmartDashboard.putNumber("Left Current", leftLead.getOutputCurrent());
        SmartDashboard.putNumber("Right Current", rightLead.getOutputCurrent());
      }
      
      @Override
      public void periodic() {
         m_pose = m_odometry.update(getHeading(), Units.feetToMeters(LeftEncoderDistance()), Units.feetToMeters(RightEncoderDistance()) );
      }
}
