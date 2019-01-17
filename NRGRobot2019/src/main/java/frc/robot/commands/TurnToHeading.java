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

public class TurnToHeading extends Command {
  private final double DEFAULT_TURN_TOLERANCE = 5.0;

  private double desiredHeading;
  private double maxPower;
  public TurnToHeading(double desiredHeading, double maxPower) {
    this.desiredHeading = desiredHeading;
    this.maxPower = Math.abs(maxPower);
    this.requires (Robot.drive);
    }
    
    // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.turnToHeadingInit(this.desiredHeading, DEFAULT_TURN_TOLERANCE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.turnToHeadingExecute(this.maxPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.turnToHeadingOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.turnToHeadingEnd();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
