/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class FollowPathWeaverFile extends FollowTrajectory{
  public FollowPathWeaverFile(String filename) {
    super(LoadTrajectoryFromCsvFile(filename));
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  
  private static Trajectory LoadTrajectoryFromCsvFile(String filename){
    File path = new File(Filesystem.getDeployDirectory(), filename);
    Trajectory pathTrajectory = Pathfinder.readFromCSV(path);
    return pathTrajectory;
  }
  // Called just before this Command runs the first time
}
