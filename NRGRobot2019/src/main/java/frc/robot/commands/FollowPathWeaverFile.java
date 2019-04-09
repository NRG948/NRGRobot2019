package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
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
    System.out.println("Loading file " + path.getAbsolutePath());

    if (!path.exists()) {
      System.out.println("ERROR: pathweaver file " + filename + " does not exist.");
      return null;
    }

    Trajectory pathTrajectory = Pathfinder.readFromCSV(path);
    return pathTrajectory;
  }

  public static String getPathWeaverFileName(AutoMovement from, AutoFeederPosition to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  public static String getPathWeaverFileName(AutoStartingPosition from, AutoMovement to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  public static String getPathWeaverFileName(AutoFeederPosition from, AutoMovement to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  public static String getPathWeaverFileName(String from, String to) {
    return "output/" + from + "_TO_" + to + ".pf1.csv";
  }
}
