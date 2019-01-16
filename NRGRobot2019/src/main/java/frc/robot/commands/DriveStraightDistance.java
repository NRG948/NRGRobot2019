/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveStraightDistance extends Command {
  private double xOrigin;
  private double yOrigin;
  private final double distance;
  private final double maxPower; 

  public DriveStraightDistance(double distance, double maxPower) {
   this.requires(Robot.drive);
   this.distance = distance;
   this.maxPower = maxPower;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.xOrigin = Robot.positionTracker.getX();
    this.yOrigin = Robot.positionTracker.getY();
    Robot.drive.driveOnHeadingInit(RobotMap.navx.getAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.driveOnHeadingExecute(this.maxPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.positionTracker.calculateDistance(this.xOrigin, this.yOrigin) >= this.distance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
