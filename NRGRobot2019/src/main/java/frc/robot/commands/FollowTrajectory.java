/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;
import jaci.pathfinder.Trajectory;
import frc.robot.Robot;

public class FollowTrajectory extends Command {

  Trajectory trajectory;
  public FollowTrajectory(Trajectory trajectory) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.trajectory = trajectory;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Follow Trajectory Init");
    Robot.drive.followTrajectoryInit(trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.followTrajectoryExecute();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.followTrajectoryIsFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.followTrajectoryEnd();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
