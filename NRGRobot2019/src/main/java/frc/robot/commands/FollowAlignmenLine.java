/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.vision.*;

public class FollowAlignmenLine extends Command {
  static ColorSensor colorSensor1 = RobotMap.colorSensor;
  static ColorSensor colorSensor2 = RobotMap.colorSensor2;
  boolean leftSensorState;
  boolean RightSensorState;
  int correctionCounter = 0;
  private double maxPower;



  public FollowAlignmenLine(double maxPower) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.maxPower = maxPower;
    requires(Robot.drive);// this uses drive subsystem 

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    UpdateSensorStates(colorSensor1, colorSensor2);
    determineHeading();
    Robot.drive.driveOnHeadingExecute(this.maxPower);


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; // has no end will run untill interrupted, will add ending code when we have WebCam code
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stopMotor();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  private void UpdateSensorStates(ColorSensor leftSensor, ColorSensor rightSensor){ // Updates sensor state vars on whitch sensors see white

    if(leftSensor.getRed() > 230 && leftSensor.getGreen() > 230 && leftSensor.getBlue() > 230)
      this.leftSensorState = true;
    else
      this.leftSensorState = false;
    if(rightSensor.getRed() > 230 && rightSensor.getGreen() > 230 && rightSensor.getBlue() > 230)
      this.RightSensorState = true;
    else
      this.RightSensorState = false;
    



  }
  private void determineHeading(){ // changes driving angle based on which sensors see white
    if(this.leftSensorState && this.RightSensorState){ // Drive straight if both sensors see white
      Robot.drive.driveOnHeadingInit (RobotMap.navx.getAngle()); 
      correctionCounter = 0;
    } 
    if(this.leftSensorState && !this.RightSensorState){ // Drive left if right sensor is off
      correctionCounter += 1;
      Robot.drive.driveOnHeadingInit (RobotMap.navx.getAngle() - (0.5 * correctionCounter));       
    }
    if(!this.leftSensorState && this.RightSensorState){ // Drive right if left sensor is off
      correctionCounter += 1;
      Robot.drive.driveOnHeadingInit (RobotMap.navx.getAngle() + (0.5 * correctionCounter)); 
    }
    if(!this.leftSensorState && this.RightSensorState){ // Find last posotive sensor state and correct based on that
      for(int i = 0;!this.leftSensorState && !this.RightSensorState; i++){
        colorSensor1.setBlock(colorSensor1.getBlockArray().get(colorSensor1.getBlockArray().size() - i));
        colorSensor1.setBlock(colorSensor2.getBlockArray().get(colorSensor2.getBlockArray().size() - i));
        UpdateSensorStates(colorSensor1, colorSensor2);
      }
      determineHeading();
    }
  }
}
