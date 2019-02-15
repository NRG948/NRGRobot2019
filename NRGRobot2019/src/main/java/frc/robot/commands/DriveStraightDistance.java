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
  private double xOrigin;// a class variable
  private double yOrigin;
  private final double distance;// this is a constant 
  private final double maxPower; 

  public DriveStraightDistance(double distance, double maxPower) {
   this.requires(Robot.drive);// requires Robot.drive in order to Drive straight
   this.distance = distance;// sets the value of the distance that we want to travel
   this.maxPower = maxPower;// sets the maximum power 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.xOrigin = Robot.positionTracker.getX(); // gets our current X position from the poistion tracker command
    this.yOrigin = Robot.positionTracker.getY();// gets our current Y position from the poistion tracker command
    Robot.drive.driveOnHeadingInit(RobotMap.navx.getAngle()); // We are getting our current heading and putting it into driveOnHeadingInit to adjust our current heading 
    System.out.println("DriveStraightDistance.init()");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.driveOnHeadingExecute(this.maxPower);// excuting the command and puts in the maximum power that the robot is gonna run on
    System.out.println("DriveStraightDistance.execute()");

  }

  @Override
  protected boolean isFinished() {
    return Robot.positionTracker.calculateDistance(this.xOrigin, this.yOrigin) >= this.distance;
  }// this calculates the distance that we have traveled from origin in order to figure out if the command 
  //needs to be terminated or not and returns true or false true being it needs to be terminated and false being it needs to be continued

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.driveOnHeadingEnd();
  } // terminated the command as the robot has reached the distance that needs to be traveled or if it needs to be interrupted

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }// calls the end command 
}
