/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//General Robot CAN and Network IDs (i.e. PCM, PDP, Raspberry Pi, etc)
	public static final int PCMChannel = 1;
	public static final int PDPChannel = 0;
	public static final String vision_PI_IP = "10.61.81.12";
	public static final String camera_PI_IP = "10.61.81.11";
	
	//Shooter
	public static final int shooterWheelID = 20;
	
	//Shooter Lift
	public static final int shooterLiftID = 22;
	public static final int shooterLiftPID = 0;
	public static final Double shooterLiftMaxLimit = 1000.0;
	public static final Double shooterLiftMinLimit = 0.0;
	public static final double ShooterJogCounts = 10.0;

	//Conveyor
	public static final int conveyorBeltsID = 23;
	public static final int conveyorGateReverseID = 7;
	public static final int conveyorGateForwardID = 0;

	//Driver Joystick
	public static final int driverJoystickID = 0;
	public static final int DRIVER_X_AXIS = 0;
	public static final int DRIVER_Y_AXIS = 1;
	public static final int DRIVER_Z_AXIS = 2;
	public static final int DRIVER_TRIGGER = 1;
	public static final int DRIVER_BUTTON2 = 3;
	public static final int DRIVER_BUTTON3 = 4;

	//Copilot Xbox Controller
	public static final int copilotControllerID = 1;
	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int LEFT_STICK_Y = 2;
	public static final int RIGHT_TRIGGER = 3;
	public static final int LEFT_TRIGGER = 4;
	public static final int RIGHT_STICK_Y = 6;
	public static final double AXIS_DEADBAND = 0.5;

	//Drivetrain Drive Motors
	public static final int FLDriveMotorID = 10;
	public static final int FRDriveMotorID = 11;
	public static final int BLDriveMotorID = 13;
	public static final int BRDriveMotorID = 12;

	//Drivetrain Steer Motors
	public static final int FLSteerMotorID = 16;
	public static final int FRSteerMotorID = 14;
	public static final int BLSteerMotorID = 15;
	public static final int BRSteerMotorID = 17;

	//Drivetrain Steer Encoders
	public static final int FLSteerEncoderID = 2;
	public static final int FRSteerEncoderID = 3;
	public static final int BLSteerEncoderID = 0;
	public static final int BRSteerEncoderID = 1;

	//Drivetrain Steer Encoder Offsets
	public static final double FLSteeringOffset = 81.0;
	public static final double FRSteeringOffset = 260.0;
	public static final double BLSteeringOffset = 74.0;
	public static final double BRSteeringOffset = 241.0;

	//Drivetrain Misc Variables and Constants
	public static final double DrivetrainGearRatio = 1 / 6.69;
	public static final double DriveWheelRadius = 2;

	//Climber
	public static final int ClimbMotor1ID = 21;
	public static final int ClimbMotor2ID = 30;
	
	public static double kPDriveVel = 1.0;
	public static double kWheelBase = 29.5;
	public static double kTrackwidth = 21.5;
	public static double kMaxSpeedMetersPerSecond = 2.0;
	public static double kPThetaController = 1.0;
	public static Constraints kThetaControllerConstraints;

	public static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(RobotMap.kWheelBase / 2, RobotMap.kTrackwidth / 2), //Front Left
      new Translation2d(RobotMap.kWheelBase / 2, -RobotMap.kTrackwidth / 2), //Front Right
      new Translation2d(-RobotMap.kWheelBase / 2, RobotMap.kTrackwidth / 2), //Back Left
      new Translation2d(-RobotMap.kWheelBase / 2, -RobotMap.kTrackwidth / 2) //Back Right
  );
	

	


}
