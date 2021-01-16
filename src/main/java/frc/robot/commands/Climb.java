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

public class Climb extends Command {
   
  public Climb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.GetCopilotRawAxis(RobotMap.RIGHT_STICK_Y) > 0.8){
      Robot.m_climber.moveclimber(1);
    }else if(Robot.m_oi.GetCopilotRawAxis(RobotMap.RIGHT_STICK_Y) > 0.2){
      Robot.m_climber.moveclimber(0.3);
    }else if(Robot.m_oi.GetCopilotRawAxis(RobotMap.RIGHT_STICK_Y) < -0.2){
      Robot.m_climber.moveclimber(-1);
    }else{
      Robot.m_climber.moveclimber(0.0);
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
    Robot.m_climber.moveclimber(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
