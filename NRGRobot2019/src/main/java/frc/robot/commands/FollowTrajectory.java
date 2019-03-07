package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;

/**
 * Follows a trajectory.
 */
public class FollowTrajectory extends Command {

  Trajectory trajectory;

  public FollowTrajectory(Trajectory trajectory) {
    requires(Robot.drive);
    this.trajectory = trajectory;
  }

  @Override
  protected void initialize() {
    Point position = Robot.positionTracker.getPosition();
    System.out.println(String.format("Follow Trajectory Init: x = %.1f, y = %.1f", position.x,position.y));
    if (trajectory != null) {
      Robot.drive.followTrajectoryInit(trajectory);
    }
  }

  @Override
  protected void execute() {
    if (trajectory != null) {
      Robot.drive.followTrajectoryExecute();
    }
  }

  @Override
  protected boolean isFinished() {
    return trajectory == null || Robot.drive.followTrajectoryIsFinished();
  }

  @Override
  protected void end() {
    if (trajectory != null) {
      Robot.drive.followTrajectoryEnd();
    }
    Point position = Robot.positionTracker.getPosition();
    System.out.println(String.format("Follow Trajectory End: x = %.1f, y = %.1f", position.x,position.y));  }

  @Override
  protected void interrupted() {
    end();
  }
}
