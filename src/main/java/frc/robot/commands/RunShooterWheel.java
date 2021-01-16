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

public class RunShooterWheel extends Command {

  private double m_run = 0.0;

  public RunShooterWheel(double Run) {

    m_run = Run;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.GetCopilotRawAxis(RobotMap.RIGHT_TRIGGER) > RobotMap.AXIS_DEADBAND){
      Robot.m_shooter.runshooterWheel(1.0);
    }else if(Math.abs(m_run) > 0.0){
      Robot.m_shooter.runshooterWheel(m_run);
    }else{
      Robot.m_shooter.runshooterWheel(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_shooter.getshooterwheelstatus() == 0.0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_shooter.runshooterWheel(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
