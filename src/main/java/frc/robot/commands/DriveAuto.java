// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveAuto extends Command {
  private Trajectory m_Trajectory;
  private SwerveControllerCommand m_SwerveControllerCommand;

  public DriveAuto(Trajectory Base) {
    m_Trajectory = Base;
    // Use requires() here to declare subsystem dependencies
    super.requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*m_SwerveControllerCommand = new SwerveControllerCommand(m_Trajectory, Robot.m_drivetrain.odometry, RobotMap.kDriveKinematics, 
    new PIDController(RobotMap.kPDriveVel, 0, 0),
    new PIDController(RobotMap.kPDriveVel, 0, 0), new PIDController(RobotMap.kPDriveVel, 0, 0), Robot.m_drivetrain.states, Robot.m_drivetrain);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
*/