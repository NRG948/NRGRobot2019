/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnToHeading extends Command {
  private final double DEFAULT_TURN_TOLERANCE = 5.0;

  private double desiredHeading; //  this gives the angle that we need to turn
  private double maxPower; // gives the power the robot is gonna drive when the command is executed 
  public TurnToHeading(double desiredHeading, double maxPower) {
    this.desiredHeading = desiredHeading;// assignes the heading 
    this.maxPower = Math.abs(maxPower);// assignes the power 
    this.requires (Robot.drive);// requires the Drive subsystem
    }
    
    // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.turnToHeadingInit(this.desiredHeading, DEFAULT_TURN_TOLERANCE); 
    // this gives in the angle into the command and intializes the command and gives in the tolerance
    System.out.println("TurnToHeading.init()"); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.turnToHeadingExecute(this.maxPower);
    System.out.println("TurnToHeading.execute()"); 
  }// this gives the power/ the speed into the command and executes it.
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.turnToHeadingOnTarget();// this command checks whether the robot is on target if not corrects it.
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.turnToHeadingEnd();// terminates the command 
  }

  @Override
  protected void interrupted() {
    end();// calls the end method
  }
}
