/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.VisionTargetsApproach;
import frc.robot.utilities.NRGPreferences.NumberPrefs;

/**
 * Add your docs here.
 */
public class DriveToVisionTapeThree extends DriveDistanceOnHeading {
  private static final double DRIVE_POWER = 0.65;
  private VisionTargetsApproach delivery;
  private Command currentCommand;

  /**
   * Add your docs here.
   */
  public DriveToVisionTapeThree(VisionTargetsApproach delivery) {
    super(0, 0, DRIVE_POWER);
    this.delivery = delivery;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.visionTargets.hasTargets()) {
      this.distanceToDrive = Robot.visionTargets.getDistanceToTarget() - this.delivery.getStopDistance();
      this.heading = Robot.visionTargets.getAngleToTarget() + RobotMap.navx.getAngle()
          + NumberPrefs.CAMERA_ANGLE_SKEW.getValue();
    } else {
      this.distanceToDrive = 0;
      this.heading = RobotMap.navx.getAngle();
    }

    super.initialize();
  }
}
