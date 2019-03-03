package frc.robot.commands;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * Command to follow all the way points in a path.
 */
public class FollowWaypoints extends FollowTrajectory {

  public static final double TIME_STEP = 0.002;
  public static final double MAX_VELOCITY = 5.6388;
  public static final double MAX_ACCELERATION = 2;
  public static final double MAX_JERK = 60;

  public FollowWaypoints(Waypoint[] waypoints) {
    super(CreateTrajectory(waypoints));
  }

  private static Trajectory CreateTrajectory(Waypoint[] waypoints) {
    var config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
        TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
    var trajectory = Pathfinder.generate(waypoints, config);
    return trajectory;
  }
}
