/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class WaitForNewVisionData extends Command {
  private int genCount = 0;
  private long startTimeNano;

  public WaitForNewVisionData() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.genCount = Robot.visionTargets.getGenCount();
    this.startTimeNano = System.nanoTime();
    System.out.println("WaitForNewVisionData init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.visionTargets.getGenCount() != this.genCount;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("WaitForNewVisionData end: duration was " + ((System.nanoTime()-this.startTimeNano)/1000000.0) + "ms");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
