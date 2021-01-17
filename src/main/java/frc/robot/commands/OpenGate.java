/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OpenGate extends Command {

  private boolean m_state = false;

  public OpenGate(Boolean state) {
    m_state = state;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_conveyorgate);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(m_state == true){
      Robot.m_conveyorgate.setgateposition(Value.kReverse); //Open Conveyor Gate to allow balls to flow into shooter
    }else{
      Robot.m_conveyorgate.setgateposition(Value.kForward); //Close Conveyor Gate to prevent balls from flowing into shooter
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
    Robot.m_conveyorgate.setgateposition(Value.kForward);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
