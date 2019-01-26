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
import frc.robot.utilities.Waypoint;
import frc.robot.utilities.WaypointPredicate;
import frc.robot.utilities.Waypoint.CoordinateType;
import frc.robot.utilities.Waypoint.WithinInches;
import frc.robot.utilities.Waypoint.WithinInchesX;
import frc.robot.utilities.Waypoint.WithinInchesY;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static final double PATH_POWER = 0.75;
  private static final Waypoint[] FOLLOW_SQUARE = new Waypoint[]{
    // new Waypoint(0, 30, CoordinateType.ABSOLUTE, 0.65, new WithinInches(10.0)),
    // new Waypoint(5, 60, CoordinateType.ABSOLUTE, 0.65, new WithinInches(10.0)),
    // new Waypoint(15, 80, CoordinateType.ABSOLUTE, 0.65, new WithinInches(10.0)),
    // new Waypoint(30, 90, CoordinateType.ABSOLUTE, 0.65, new WithinInches(10.0)),
    // new Waypoint(50, 90, CoordinateType.ABSOLUTE, 0.65, new WithinInches(10.0)),
    new Waypoint(0, 78, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesY(10.0)),
    new Waypoint(4, 112, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesY(10.0)),
    new Waypoint(6, 126, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesY(10.0)),
    new Waypoint(10, 138, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesY(10.0)),
    new Waypoint(16, 150, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesY(10.0)),
    new Waypoint(20, 162, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInches(10.0)),
    new Waypoint(32, 174, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInches(10.0)),
    new Waypoint(42, 186, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInches(10.0)),
    new Waypoint(66, 198, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesX(10.0)),
    new Waypoint(86, 204, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesX(10.0)),
    new Waypoint(100, 208, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesX(10.0)),
    new Waypoint(120, 210, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesX(10.0)),
    new Waypoint(132, 210, CoordinateType.ABSOLUTE, PATH_POWER, new WithinInchesX(10.0)),
  };

  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);
  // assign each side of joystick to a port 

  private JoystickButton resetSensorsButton = new JoystickButton(leftJoystick, 11);
  private JoystickButton driveStraightButton = new JoystickButton(rightJoystick, 1);
  private JoystickButton turnToHeadingButton = new JoystickButton(rightJoystick, 3);
  private JoystickButton driveStraightDistanceButton = new JoystickButton(rightJoystick, 8);
  private JoystickButton followWaypointsButton = new JoystickButton(rightJoystick, 9);

  

  OI() {
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      RobotMap.resetSensors();
      Robot.positionTracker.setPosition(0.0, 0.0);
    }));

    driveStraightButton.whenActive(new DriveStraight());
    driveStraightButton.whenInactive(new ManualDrive());
    turnToHeadingButton.whenPressed(new TurnToHeading(90, 1.0));
    driveStraightDistanceButton.whenPressed(new DriveStraightDistance(120, 0.5));
    followWaypointsButton.whenPressed(new FollowWaypoints(FOLLOW_SQUARE));
  }


  public double getLeftJoystickY() {
    return -leftJoystick.getY();
  }// gets the Y value of the left joystick

  public double getRightJoystickY() {
    return -rightJoystick.getY();
  }// gets the Y value of the right joystick

}
