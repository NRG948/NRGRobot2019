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
import frc.robot.utilities.Deliver;

/**
 * Add your docs here.
 */
public class DriveToVisionTapeThree extends InstantCommand {
  private static final double DRIVE_POWER = 0.7;
  private static final double CAMERA_SKEW = -4.0;
  private Deliver delivery;

  /**
   * Add your docs here.
   */
  public DriveToVisionTapeThree(Deliver delivery) {
    super();
    this.delivery = delivery;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.visionTargets.hasTargets()) {
      double distanceToDrive = Robot.visionTargets.getDistanceToTarget() - this.delivery.getStopDistance();
      double heading = Robot.visionTargets.getAngleToTarget() + RobotMap.navx.getAngle() + CAMERA_SKEW;

      new DriveOnHeadingDistance(heading, distanceToDrive, DRIVE_POWER).start();
    }
  }
}
