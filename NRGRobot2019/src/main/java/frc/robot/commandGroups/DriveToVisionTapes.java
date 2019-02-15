/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.TurnToHeading;

public class DriveToVisionTapes extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DriveToVisionTapes() {
    // System.out.println("DriveToVisionTapes Start");
    // addSequential(new TurnToHeading(Robot.vp.getAngleToTurn(Robot.leftTarget, Robot.rightTarget), 0.5)); 
    // addSequential(new DriveStraightDistance(Robot.vp.getDistanceToCenter(Robot.leftTarget, Robot.rightTarget), 0.5));
  }
}
