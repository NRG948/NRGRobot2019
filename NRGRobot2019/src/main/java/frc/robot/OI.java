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
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static final Waypoint[] FOLLOW_SQUARE = new Waypoint[]{
    // new Waypoint(5, 0, Math.toRadians(90)),
    // new Waypoint(5, 5, Math.toRadians(180)),
    // new Waypoint(0, 5, Math.toRadians(270)),
    // new Waypoint(0, 0, Math.toRadians(360))
    new Waypoint(-4, -1, Pathfinder.d2r(-45)),
    new Waypoint(-2, -2, 0),
    new Waypoint(0, 0, 0)
  };

  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);
  // assign each side of joystick to a port 

  private JoystickButton resetSensorsButton = new JoystickButton(leftJoystick, 11);
  private JoystickButton driveStraightButton = new JoystickButton(rightJoystick, 1);
  private JoystickButton turnToHeadingButton = new JoystickButton(rightJoystick, 3);
  private JoystickButton driveStraightDistanceButton = new JoystickButton(rightJoystick, 8);
  private JoystickButton followSquareButton = new JoystickButton(rightJoystick, 9);


  

  OI() {
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      RobotMap.resetSensors();
      Robot.positionTracker.setPosition(0.0, 0.0);
    }));

    driveStraightButton.whenActive(new DriveStraight());
    driveStraightButton.whenInactive(new ManualDrive());
    turnToHeadingButton.whenPressed(new TurnToHeading(90, 1.0));
    driveStraightDistanceButton.whenPressed(new DriveStraightDistance(120, 0.5));
    // followSquareButton.whenPressed(new FollowWaypoints(FOLLOW_SQUARE));
  }


  public double getLeftJoystickY() {
    return -leftJoystick.getY();
  }// gets the Y value of the left joystick

  public double getRightJoystickY() {
    return -rightJoystick.getY();
  }// gets the Y value of the right joystick

}
