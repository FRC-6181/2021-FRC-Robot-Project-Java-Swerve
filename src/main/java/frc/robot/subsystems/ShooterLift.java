/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShooterLift extends Subsystem {

  private TalonSRX shooterLift = new TalonSRX(RobotMap.shooterLiftID);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public ShooterLift() {
    this.shooterLift.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, RobotMap.shooterLiftPID, 0);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getshooterPosition(){
    return shooterLift.getSelectedSensorPosition();
  }

  public void setShooterPosition(Double Position) {
    shooterLift.set(TalonSRXControlMode.Position, Position);
  }

}
