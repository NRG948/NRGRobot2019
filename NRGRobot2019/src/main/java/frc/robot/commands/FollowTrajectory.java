package frc.robot.commands;

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
    System.out.println("Follow Trajectory Init");
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
    System.out.println("Follow Trajectory End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
