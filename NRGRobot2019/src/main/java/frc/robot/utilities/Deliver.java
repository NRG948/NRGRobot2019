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
public enum Deliver {
  Hatch(12.25), Cargo(34),
  HatchPickUp(6.25);//CHANGE BACK TO 10

  private final double stopDistance;

  private Deliver(double stopDistance) {
    this.stopDistance = stopDistance;
  }

  public double getStopDistance() {
    return this.stopDistance;
  }
}
