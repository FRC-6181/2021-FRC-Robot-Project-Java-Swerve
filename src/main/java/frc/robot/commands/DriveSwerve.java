/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveSwerve extends Command {
  public DriveSwerve() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.GetDriverRawButton(RobotMap.DRIVER_BUTTON2) == true){
      Robot.m_drivetrain.drive(Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_X_AXIS), 
      Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_Y_AXIS), 
      -0.5, 
      Robot.m_oi.GetDriverRawButton(RobotMap.DRIVER_TRIGGER));
    }else if(Robot.m_oi.GetDriverRawButton(RobotMap.DRIVER_BUTTON3) == true){
      Robot.m_drivetrain.drive(Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_X_AXIS), 
      Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_Y_AXIS), 
      0.5, 
      Robot.m_oi.GetDriverRawButton(RobotMap.DRIVER_TRIGGER));
    }else{    
      Robot.m_drivetrain.drive(Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_X_AXIS), 
      Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_Y_AXIS), 
      Robot.m_oi.GetDriverRawAxis(RobotMap.DRIVER_Z_AXIS), 
      Robot.m_oi.GetDriverRawButton(RobotMap.DRIVER_TRIGGER));
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_drivetrain.drive(0, 0, 0, false); //Prevent a robot run away incase of communication interuption
  }
}
