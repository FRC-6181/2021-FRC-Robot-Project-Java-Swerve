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

public class RunConveyor extends Command {

  private double m_speed;

  public RunConveyor(Double Speed) {
    m_speed = Speed;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_conveyorbelts);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.GetCopilotRawAxis(RobotMap.RIGHT_TRIGGER) > RobotMap.AXIS_DEADBAND){
      Robot.m_conveyorbelts.setbeltspeed(1.0);
    }else if(Math.abs(m_speed) > 0){
      Robot.m_conveyorbelts.setbeltspeed(m_speed);
    }else{
      Robot.m_conveyorbelts.setbeltspeed(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_conveyorbelts.getconveyorstatus() == 0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_conveyorbelts.setbeltspeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
