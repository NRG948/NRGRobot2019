/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.TurnToHeading;

public class FeederStationToCargoFirstHatchClose extends CommandGroup {
  /**
   * Drive from feeder station to cargo first close and stick the hatch.
   */
  public FeederStationToCargoFirstHatchClose(AutoFeederPosition position, double drivePower, double turnPower) {
    double heading = position == AutoFeederPosition.RIGHT_FEEDER ? -190 : 190;
    addSequential(new DriveDistanceOnHeading(heading, 240, drivePower, 2.0));
    heading = position == AutoFeederPosition.RIGHT_FEEDER ? -85 : 85;
    addSequential(new TurnToHeading(heading, turnPower));
    addSequential(new DeliverHatchWithoutRelease());
  }
}
