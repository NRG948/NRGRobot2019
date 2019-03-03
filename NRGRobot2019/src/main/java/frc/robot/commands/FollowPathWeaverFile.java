package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Following the trajectories from the pathweaver file.
 */
public class FollowPathWeaverFile extends FollowTrajectory {

  public FollowPathWeaverFile(String filename) {
    super(LoadTrajectoryFromCsvFile(filename));
    requires(Robot.drive); // this command only uses the drive subsystem
  }

  private static Trajectory LoadTrajectoryFromCsvFile(String filename) {
    File path = new File(Filesystem.getDeployDirectory(), filename);
    Trajectory pathTrajectory = Pathfinder.readFromCSV(path);
    return pathTrajectory;
  }
}
