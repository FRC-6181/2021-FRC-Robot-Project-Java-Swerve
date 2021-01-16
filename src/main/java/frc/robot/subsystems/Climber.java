/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

  private CANSparkMax ClimbMotor1 = new CANSparkMax(RobotMap.ClimbMotor1ID, MotorType.kBrushless);
  private CANSparkMax ClimbMotor2 = new CANSparkMax(RobotMap.ClimbMotor2ID, MotorType.kBrushless);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Climber(){
    ClimbMotor2.follow(ClimbMotor1);
    ClimbMotor1.setSoftLimit(SoftLimitDirection.kForward, 80);
    ClimbMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveclimber(double speed){
    ClimbMotor1.set(speed);
  }
}
