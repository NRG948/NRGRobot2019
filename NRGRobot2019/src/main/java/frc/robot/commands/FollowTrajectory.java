package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;

/**
 * Follows a trajectory.
 */
public class FollowTrajectory extends Command {

  Trajectory left;
  Trajectory right;

  public FollowTrajectory(Trajectory left, Trajectory right) {
    requires(Robot.drive);
    this.left = left;
    this.right = right;
  }

  @Override
  protected void initialize() {
    Point position = Robot.positionTracker.getPosition();
    System.out.println(String.format("Follow Trajectory Init: x = %.1f, y = %.1f", position.x,position.y));
    if (left != null) {
      Robot.drive.followTrajectoryInit(left, right);
    }
  }

  @Override
  protected void execute() {
    if (left != null) {
      Robot.drive.followTrajectoryExecute();
    }
  }

  @Override
  protected boolean isFinished() {
    return left == null || Robot.drive.followTrajectoryIsFinished();
  }

  @Override
  protected void end() {
    if (left != null) {
      Robot.drive.followTrajectoryEnd();
    }
    Point position = Robot.positionTracker.getPosition();
    System.out.println(String.format("Follow Trajectory End: x = %.1f, y = %.1f", position.x,position.y));  }

  @Override
  protected void interrupted() {
    end();
  }
}
