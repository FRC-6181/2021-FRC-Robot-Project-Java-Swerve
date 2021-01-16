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


public class MoveShooter extends Command {

  private double m_value = 0.0;
  private double m_currentPosition = 0.0;

  public MoveShooter(Double Value) {
    m_value = Value;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_shooterlift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    
    if(Robot.m_oi.GetCopilotRawAxis(RobotMap.LEFT_STICK_Y) > RobotMap.AXIS_DEADBAND){
      m_currentPosition = m_currentPosition + RobotMap.ShooterJogCounts;
      if(m_currentPosition < RobotMap.shooterLiftMaxLimit && m_currentPosition > RobotMap.shooterLiftMinLimit){
        Robot.m_shooterlift.setShooterPosition(m_value);
      }else{
        Robot.m_shooterlift.setShooterPosition(RobotMap.shooterLiftMaxLimit);
      }
    }else if(Robot.m_oi.GetCopilotRawAxis(RobotMap.LEFT_STICK_Y) < 0){
      m_currentPosition = m_currentPosition - RobotMap.ShooterJogCounts;
      if(m_currentPosition < RobotMap.shooterLiftMaxLimit && m_currentPosition > RobotMap.shooterLiftMinLimit){
        Robot.m_shooterlift.setShooterPosition(m_currentPosition);
      }else{
        Robot.m_shooterlift.setShooterPosition(RobotMap.shooterLiftMinLimit);
      }
    }else{
      m_currentPosition = m_value;
      Robot.m_shooterlift.setShooterPosition(m_currentPosition);
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
  }
}
