/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ConveyorGate extends Subsystem {

  private DoubleSolenoid conveyorGate = new DoubleSolenoid(RobotMap.PCMChannel, RobotMap.conveyorGateForwardID, RobotMap.conveyorGateReverseID);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void openconveyorGate(Boolean Open){
    if(Open == true){
      conveyorGate.set(Value.kReverse);
    }
    else{
      conveyorGate.set(Value.kForward);
    }
  }

  public void setgateposition(Value Position){
    conveyorGate.set(Position);
  }

  public Value getgateposition() {
    return conveyorGate.get();
  }

}
