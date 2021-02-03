/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.RobotMap;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain extends Subsystem{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private double[] WheelAngles = {0, 0, 0, 0};

  private final SwerveModule m_frontLeft = new SwerveModule(
    RobotMap.FLDriveMotorID, 
    RobotMap.FLSteerMotorID, 
    RobotMap.FLSteerEncoderID, 
    RobotMap.FLSteeringOffset);
  private final SwerveModule m_frontRight = new SwerveModule(
    RobotMap.FRDriveMotorID, 
    RobotMap.FRSteerMotorID, 
    RobotMap.FRSteerEncoderID, 
    RobotMap.FRSteeringOffset);
  private final SwerveModule m_backLeft = new SwerveModule(
    RobotMap.BLDriveMotorID, 
    RobotMap.BLSteerMotorID, 
    RobotMap.BLSteerEncoderID, 
    RobotMap.BLSteeringOffset);
  private final SwerveModule m_backRight = new SwerveModule(
    RobotMap.BRDriveMotorID, 
    RobotMap.BRSteerMotorID, 
    RobotMap.BRSteerEncoderID, 
    RobotMap.BRSteeringOffset);

  private final AHRS m_gyro = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(RobotMap.kWheelBase / 2, RobotMap.kTrackwidth / 2), //Front Left
      new Translation2d(RobotMap.kWheelBase / 2, -RobotMap.kTrackwidth / 2), //Front Right
      new Translation2d(-RobotMap.kWheelBase / 2, RobotMap.kTrackwidth / 2), //Back Left
      new Translation2d(-RobotMap.kWheelBase / 2, -RobotMap.kTrackwidth / 2) //Back Right
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Returns the Current Pose of the Robot
   * 
   * @return Current Pose
   */
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the Current Angles of the Swerve Modules
   * @return Wheel Module Angles
   */
  public double[] getModuleAngle(){
    this.WheelAngles[0] = m_frontLeft.getwheelposition();
    this.WheelAngles[1] = m_frontRight.getwheelposition();
    this.WheelAngles[2] = m_backLeft.getwheelposition();
    this.WheelAngles[3] = m_backRight.getwheelposition();
    return WheelAngles;
  }

  /**
   * Resets the Pose to the Trajectory Starting Point
   * 
   * @param Pose Starting Point of the Trajectory
   */
  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

    /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, RobotMap.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
  

  @Override
  protected void initDefaultCommand() {

  }
}
