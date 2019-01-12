/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);

  private JoystickButton resetSensorsButton = new JoystickButton(leftJoystick, 11);

  OI() {
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      RobotMap.resetSensors();
    }));
  }

  public double getLeftJoystickY() {
    return -leftJoystick.getY();
  }

  public double getRightJoystickY() {
    return -rightJoystick.getY();
  }

}
