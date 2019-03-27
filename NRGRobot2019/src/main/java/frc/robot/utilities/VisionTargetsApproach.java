/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * Add your docs here.
 */
public enum VisionTargetsApproach {
  Cargo(34),
  HatchDeliver(12.25),
  HatchPickUp(13.5 - 6.25);

  private final double stopDistance;

  private VisionTargetsApproach(double stopDistance) {
    this.stopDistance = stopDistance;
  }

  public double getStopDistance() {
    return this.stopDistance;
  }
}
