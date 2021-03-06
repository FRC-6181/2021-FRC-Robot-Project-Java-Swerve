/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.OpenGate;
import frc.robot.commands.RunConveyor;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private Joystick driverJoystick = new Joystick(RobotMap.driverJoystickID);

  private XboxController copilotController = new XboxController(RobotMap.copilotControllerID);
  private Button xButton = new JoystickButton(copilotController, RobotMap.BUTTON_X);
  private Button bButton = new JoystickButton(copilotController, RobotMap.BUTTON_B);
  private Button yButton = new JoystickButton(copilotController, RobotMap.BUTTON_Y);
  private Button aButton = new JoystickButton(copilotController, RobotMap.BUTTON_A);
  
  public double GetDriverRawAxis(int axis) { //gets the Raw Axis Value from the Driver Joystick
    return driverJoystick.getRawAxis(axis);
  }

  public boolean GetDriverRawButton(int Button) { //gets the Raw Button Value from the Driver Joystick
    return driverJoystick.getRawButton(Button);
  }

  public double GetCopilotRawAxis(int axis) { //gets the Raw Axis Value from the Copilot Joystick
    return copilotController.getRawAxis(axis);
  }

  public boolean GetCopilotRawButton(int Button){ //gets the Raw Button Value from the Copilot Joystick
    return copilotController.getRawButton(Button);
  }

  public OI(){
    
    bButton.whileHeld(new OpenGate(true)); //Opens the Conveyor Gate

    yButton.whileHeld(new RunConveyor(-1.0)); //Runs the Conveyor in Reverse
    
    //The Following Controls Jog the Shooter into different Positions
    aButton.whenPressed(new MoveShooter(0.0));
    xButton.whenPressed(new MoveShooter(750.0));
    
  }
  
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
