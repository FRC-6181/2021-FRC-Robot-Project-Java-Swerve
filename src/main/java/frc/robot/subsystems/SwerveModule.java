/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.RobotMap;

public class SwerveModule{
  
  private static double m_turningoffset;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final VictorSPX m_turningMotor;

  private final AnalogEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param turningEncoderPort Port ID for Turning Encoder.
   * @param wheelOffset Starting Point of the Wheel when the encoder is at Zero.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderPort, double wheelOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless); // Sets the Drive Motor ID and Motor Type
    m_turningMotor = new VictorSPX(turningMotorChannel); // Sets the Turn Motor ID
    m_turningEncoder = new AnalogEncoder(new AnalogInput(turningEncoderPort)); // Sets the Turn Encoder Port
    m_turningoffset = wheelOffset; // Sets the offset for the Turning Encoder
    
    m_turningEncoder.setDistancePerRotation(360.0);

    m_driveMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * RobotMap.DrivetrainGearRatio * Units.inchesToMeters(RobotMap.DriveWheelRadius));
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity(),
        new Rotation2d(Units.degreesToRadians(m_turningEncoder.get() + m_turningoffset)));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(Units.degreesToRadians(m_turningEncoder.get() + m_turningoffset)));
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(
      m_driveMotor.getEncoder().getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(
      Units.degreesToRadians(m_turningEncoder.get() + m_turningoffset), state.angle.getRadians()
    );

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,turnOutput);
  }

  public double getwheelposition(){
    return m_turningEncoder.get();
  }

}
