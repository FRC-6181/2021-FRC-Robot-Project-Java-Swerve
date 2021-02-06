/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ConveyorBelts extends Subsystem {

  private TalonSRX conveyorBelts = new TalonSRX(RobotMap.conveyorBeltsID);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new RunConveyor(0.0));
  }

  public double getconveyorstatus(){
    return conveyorBelts.getMotorOutputPercent();
  }

  public void setbeltspeed(Double Speed){
    conveyorBelts.set(TalonSRXControlMode.PercentOutput, Speed);
  }

}
