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

public class DriveStraight extends Command {
  public DriveStraight() {
    this.requires(Robot.drive);
    // this command requires the drive subsystem 
    //in order to function as all the PID values are defined there
  }

  @Override
  protected void initialize() {
    System.out.println("Drive Strait Init");
      Robot.drive.driveOnHeadingInit (RobotMap.navx.getAngle ()); 
      // getting the current driving angle from the gyro 
  }
  @Override
  protected void execute() {
      Robot.drive.driveOnHeadingExecute (Robot.oi.getRightJoystickY());
  }
    // we are excuting driveOnHeadingExecute command and we are getting the Y joystick value so that we can drive forward as the right joystick drives the robot forward.
  @Override
  protected boolean isFinished() {
    return false;
  }
  // we return false here as we dont want this command to stop since we want our robot to drive straight!

  // Called once after isFinished returns true
  @Override
  protected void end() {
      Robot.drive.driveOnHeadingEnd ();
      // we terminate this command using driveOnHeadingEnd
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
      // calls the end method 
  }
}
