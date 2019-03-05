/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveToVisionTapeThree extends InstantCommand {
  private static final double DRIVE_POWER = 0.7;

  /**
   * Add your docs here.
   */
  public DriveToVisionTapeThree() {
    super();

  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.visionTargets.hasTargets()) {
      double distanceToDrive = Robot.visionTargets.getDistanceToTarget();
      double heading = Robot.visionTargets.getAngleToTarget() + RobotMap.navx.getAngle();

      new DriveOnHeadingDistance(heading, distanceToDrive, DRIVE_POWER).start();
    }
  }
}
